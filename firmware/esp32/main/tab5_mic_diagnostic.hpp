// Microphone-only Tab5 diagnostic: the pure half, host-testable.
//
// No ESP-IDF, I2S or codec dependency lives here: the ESP adapter in
// tab5_mic_diagnostic.cpp feeds interleaved PCM16 blocks in and prints the
// lines this header formats. Level math reuses hg::rms/level_percent
// (hg/vad.hpp), so the probe reports the same numbers the App's VAD sees.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "hg/vad.hpp"

namespace hgp {

// Same rate as CodecAudio::kRate: the ES7210 and every SDK codec board run at
// 24 kHz, and a level reading at another rate is not comparable.
inline constexpr uint32_t kTab5MicRate = 24000;
// One report line every 250 ms is enough to see speech, and bounded: at most
// 120 lines in a whole session.
inline constexpr uint32_t kTab5MicReportMs = 250;
// The probe releases the ADC after 30 s; running it again needs a new boot.
inline constexpr uint32_t kTab5MicSessionMs = 30000;
inline constexpr size_t kTab5MicBarCells = 12;
// Full scale. A block that reaches it is counted as clipped, so a probe that
// only ever sees 0 or 100 is distinguishable from a quiet room.
inline constexpr int32_t kTab5MicClipLevel = 32767;

// What the ESP adapter captures the ES7210 as: interleaved MIC1|MIC2 slots.
struct Tab5MicFormat {
  uint32_t sample_rate;
  uint16_t channels;
  uint16_t bits;
};

inline constexpr Tab5MicFormat tab5_mic_format() {
  return Tab5MicFormat{kTab5MicRate, 2, 16};
}

// Accumulated over every block of one session. Nothing here is persisted or
// transmitted: it exists to print one summary line at the end.
struct Tab5MicStats {
  uint64_t frames = 0;       // mono frames seen
  double sum_squares = 0;    // over every mono sample, for the session RMS
  int32_t peak = 0;          // largest absolute mono sample, clamped to 32767
  float block_rms_peak = 0;  // loudest single block
  uint32_t blocks = 0;
  uint32_t clipped_blocks = 0;

  void add(const int16_t* mono, size_t count) {
    if (!mono || count == 0) return;
    double block = 0;
    bool clipped = false;
    for (size_t i = 0; i < count; ++i) {
      const int32_t sample = mono[i];
      block += double(sample) * double(sample);
      // -32768 is one step past full scale: clamp so the peak stays reportable.
      const int32_t magnitude = sample < 0 ? -sample : sample;
      if (magnitude > peak) peak = magnitude > kTab5MicClipLevel ? kTab5MicClipLevel : magnitude;
      if (magnitude >= kTab5MicClipLevel) clipped = true;
    }
    sum_squares += block;
    frames += count;
    ++blocks;
    const float block_rms = static_cast<float>(std::sqrt(block / double(count)));
    if (block_rms > block_rms_peak) block_rms_peak = block_rms;
    if (clipped) ++clipped_blocks;
  }

  float rms() const {
    if (!frames) return 0.0f;
    return static_cast<float>(std::sqrt(sum_squares / double(frames)));
  }
  uint8_t peak_percent() const { return hg::level_percent(static_cast<float>(peak)); }
};

// Reduces the ES7210's interleaved slots to one mono block, averaging each
// frame's channels. Never reads past `frames * channels`, and writes at most
// `capacity` frames; returns how many whole frames were written.
inline size_t tab5_mic_to_mono(const int16_t* interleaved, size_t frames, size_t channels,
                               int16_t* mono, size_t capacity) {
  if (!interleaved || !mono || channels == 0 || frames == 0 || capacity == 0) return 0;
  if (capacity < frames) frames = capacity;
  for (size_t frame = 0; frame < frames; ++frame) {
    int32_t sum = 0;
    for (size_t channel = 0; channel < channels; ++channel) sum += interleaved[frame * channels + channel];
    mono[frame] = static_cast<int16_t>(sum / static_cast<int32_t>(channels));
  }
  return frames;
}

// "mic  42% #####-------": the value and a short bar. Returns the length, or 0
// when the line does not fit — a truncated meter is worse than none.
inline size_t tab5_mic_level_line(char* out, size_t capacity, uint8_t percent) {
  if (!out || capacity == 0) return 0;
  // Rounded, so a non-zero level is never drawn as an empty bar.
  size_t filled = (size_t(percent) * kTab5MicBarCells + 50) / 100;
  if (filled > kTab5MicBarCells) filled = kTab5MicBarCells;
  char bar[kTab5MicBarCells + 1];
  for (size_t i = 0; i < kTab5MicBarCells; ++i) bar[i] = i < filled ? '#' : '-';
  bar[kTab5MicBarCells] = '\0';
  const int written = std::snprintf(out, capacity, "mic %3u%% %s", unsigned(percent), bar);
  if (written < 0 || size_t(written) >= capacity) return 0;
  return size_t(written);
}

// The one summary line at the end of the session: peak, session RMS, the
// loudest block, how much was heard, and whether anything clipped.
inline size_t tab5_mic_summary_line(char* out, size_t capacity, const Tab5MicStats& stats) {
  if (!out || capacity == 0) return 0;
  const int written = std::snprintf(
      out, capacity, "mic done: peak %d rms %.1f max-block-rms %.1f frames %llu blocks %u clipped %u",
      int(stats.peak), double(stats.rms()), double(stats.block_rms_peak),
      static_cast<unsigned long long>(stats.frames), unsigned(stats.blocks),
      unsigned(stats.clipped_blocks));
  if (written < 0 || size_t(written) >= capacity) return 0;
  return size_t(written);
}

// True while the session may still capture; a backwards clock closes it.
inline bool tab5_mic_session_open(uint64_t start_ms, uint64_t now_ms) {
  return now_ms >= start_ms && now_ms - start_ms < kTab5MicSessionMs;
}

// Moves the report deadline only when a report is actually due, so a slow read
// does not swallow the next window.
inline bool tab5_mic_report_due(uint64_t now_ms, uint64_t& last_report_ms) {
  if (now_ms < last_report_ms) return false;
  if (now_ms - last_report_ms < kTab5MicReportMs) return false;
  last_report_ms = now_ms;
  return true;
}

// The ESP adapter: RX-only I2S plus the ES7210, then the bounded session. It
// prints its summary, releases the ADC and never returns, so a repeated
// invocation needs a new boot/reset. Never constructs hg::App, NVS, network,
// OTA or an identity, and never opens the DAC or the amplifier.
[[noreturn]] void tab5_mic_diagnostic_run();

}  // namespace hgp
