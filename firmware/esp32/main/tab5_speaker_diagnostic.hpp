// Speaker-only Tab5 diagnostic: the pure half, host-testable.
//
// No ESP-IDF, I2S or codec dependency lives here: the ESP adapter in
// tab5_speaker_diagnostic.cpp drives the ES8388 (DAC only) and the 0x43
// expander, and prints the step lines this header formats. The adapter is the
// exclusive owner of that expander: no BSP expander constructor, cache, display
// or touch owner may coexist with it.
//
// The mode is deliberately narrow: one 1 kHz, 250 ms, about -40 dBFS tone
// generated here, written once over a TX-only I2S channel, with the amplifier
// enable requested off, then enabled, then off again through masked
// read-modify-writes that are read back. It never opens the ES7210 or any ADC,
// never stores or streams audio, and never constructs hg::App, NVS, network,
// OTA, an identity, the display, or the host seed exchange.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace hgp {

// Same rate as CodecAudio::kRate, the rate every SDK codec board runs at.
inline constexpr uint32_t kTab5SpeakerRate = 24000;
// One short tone: a thousand cycles of 1 kHz inside a quarter of a second.
inline constexpr uint32_t kTab5SpeakerToneHz = 1000;
inline constexpr uint32_t kTab5SpeakerToneMs = 250;
inline constexpr size_t kTab5SpeakerToneFrames =
    static_cast<size_t>(kTab5SpeakerRate) / 1000 * kTab5SpeakerToneMs;  // 6000
// 32768 * 10^(-40/20) = 327.7, rounded up: about -40 dBFS, quiet by design.
inline constexpr int16_t kTab5SpeakerTonePeak = 328;
// Percentage for esp_codec_dev_set_out_vol. The probe stays conservative: half
// scale or less, on top of the quiet tone.
inline constexpr uint8_t kTab5SpeakerVolume = 40;
static_assert(kTab5SpeakerVolume <= 50, "the speaker probe needs a conservative output volume");

// Millisecond budgets. Every IO method must honour its own budget, including
// its internal locks and retries, and the controller re-checks the elapsed time
// because an adapter may return late.
inline constexpr uint32_t kTab5SpeakerIoMs = 50;         // one expander/register operation
inline constexpr uint32_t kTab5SpeakerDacMs = 500;       // ES8388 open or close
// esp_codec_dev's I2S write waits at most 1000 ms internally; the budget can
// only detect a later return, never interrupt it.
inline constexpr uint32_t kTab5SpeakerToneWriteMs = 1000;

// The amplifier enable: expander 0x43, pin 1, active high. The registers are
// the ones tab5_prepare_speaker_off documents (0x05, 0x07 then 0x03) and only
// that pin's bit is ever changed relative to each read.
inline constexpr uint8_t kTab5SpeakerExpanderAddr = 0x43;
inline constexpr uint8_t kTab5SpeakerPaPin = 1;
inline constexpr uint8_t kTab5SpeakerMask = uint8_t(1u << kTab5SpeakerPaPin);  // 0x02
inline constexpr uint8_t kTab5SpeakerConfigReg = 0x05;
inline constexpr uint8_t kTab5SpeakerDirectionReg = 0x07;
inline constexpr uint8_t kTab5SpeakerLevelReg = 0x03;

// Unknown is sticky after any failed operation, even if a later cleanup is
// acknowledged. SafeOff is the only state that claims "the amplifier is off",
// and it is reached only after every request was acknowledged and read back.
enum class Tab5SpeakerState { Unknown, Ready, Playing, SafeOff };

inline const char* tab5_speaker_state_name(Tab5SpeakerState state) {
  switch (state) {
    case Tab5SpeakerState::Ready: return "Ready";
    case Tab5SpeakerState::Playing: return "Playing";
    case Tab5SpeakerState::SafeOff: return "SafeOff";
    case Tab5SpeakerState::Unknown: break;
  }
  return "Unknown";
}

// "speaker probe: result SafeOff" (or Unknown): the last line, always printed.
// Returns the length, or 0 when the line does not fit.
inline size_t tab5_speaker_result_line(char* out, size_t capacity, Tab5SpeakerState state) {
  if (!out || capacity == 0) return 0;
  const int written =
      std::snprintf(out, capacity, "speaker probe: result %s", tab5_speaker_state_name(state));
  if (written < 0 || size_t(written) >= capacity) return 0;
  return size_t(written);
}

// The single tone: a 1 kHz square wave at an explicit amplitude, 12 frames high
// then 12 low (one period at 24 kHz). Writes at most `capacity` frames and
// returns how many it wrote; never past kTab5SpeakerToneFrames. The shape and
// the length are shared, so the interactive probe cannot invent a second
// waveform while this file's own conservative level stays the default.
inline size_t tab5_speaker_tone(int16_t* out, size_t capacity, int16_t peak) {
  if (!out || capacity == 0 || peak <= 0) return 0;
  const size_t frames = capacity < kTab5SpeakerToneFrames ? capacity : kTab5SpeakerToneFrames;
  constexpr size_t kHalfPeriod = kTab5SpeakerRate / kTab5SpeakerToneHz / 2;  // 12 frames
  for (size_t i = 0; i < frames; ++i)
    out[i] = (i % (2 * kHalfPeriod)) < kHalfPeriod ? peak : int16_t(-peak);
  return frames;
}

// The same tone at kTab5SpeakerTonePeak: the speaker-only probe's quiet level.
inline size_t tab5_speaker_tone(int16_t* out, size_t capacity) {
  return tab5_speaker_tone(out, capacity, kTab5SpeakerTonePeak);
}

// Nothing plays at boot without a deliberate start, asked for over USB
// Serial/JTAG: 's'/'S' starts the single tone, CR/LF is ignored, and any other
// byte refuses it.
enum class Tab5SpeakerStart { Ignore, Start, Refuse };

inline Tab5SpeakerStart tab5_speaker_start_byte(int byte) {
  if (byte == '\r' || byte == '\n') return Tab5SpeakerStart::Ignore;
  if (byte == 's' || byte == 'S') return Tab5SpeakerStart::Start;
  return Tab5SpeakerStart::Refuse;
}

// Which way one attempt drives the amplifier control. High is the BSP's own
// enable convention (BSP_FEATURE_SPEAKER); Low is the other candidate. Both are
// written through the same masked read-modify-write and read back, so a probe
// pair differs by exactly one bit of one register and by nothing else.
enum class Tab5SpeakerPa : uint8_t { High, Low };

// Measured on this unit through an audible A/B probe: with the control driven
// HIGH the operator heard the tone, with it LOW the amplifier stayed silent. So
// HIGH is the enable convention every default path uses; Low stays expressible
// (the write path, the probe and tab5_audio_pa_text all accept it) but nothing
// plays it automatically any more.
inline constexpr Tab5SpeakerPa kTab5SpeakerConfirmedDrive = Tab5SpeakerPa::High;

// The polarity probe plays the tone twice, first High then Low.
inline constexpr size_t kTab5SpeakerProbeAttempts = 2;

// Called once per attempt, after that attempt's control value was written and
// read back and before its tone is queued: the adapter shows the label here and
// may hold the silent gap before the second attempt. Must not throw and must
// return promptly; it runs inside the sequence's bounds.
using Tab5SpeakerAttemptHook = void (*)(void* ctx, Tab5SpeakerPa pa);

// IO must be the exclusive, non-reentrant owner of the Tab5 0x43 expander
// (I2C SDA31/SCL32). `read`/`write` operate on that expander only. `open_dac`
// opens the ES8388 in DAC-only mode with the given volume and muted, and fails
// rather than silently unmuting; there is no ADC, no ES7210 and no GPIO
// amplifier pin. `tone` writes the whole block once over TX only and succeeds
// only if the codec accepted all of it inside the budget: that means queued,
// NOT that the speaker played it. `close_dac` releases the ES8388.
//
// The contract for every method includes finishing inside its own millisecond
// budget. esp_codec_dev_open/close use 100 ms per internal I2C transaction and
// esp_codec_dev_write waits at most 1000 ms for the I2S ring; i2s_channel_disable
// has no timeout of its own, so the adapter documents it instead of bounding it.
// This controller cannot interrupt a blocked driver: a call that never returns
// leaves the amplifier state unknown and the final line unprinted.
template<class IO>
class Tab5SpeakerDiagnostic {
 public:
  explicit Tab5SpeakerDiagnostic(IO& io) : io_(io) {}
  Tab5SpeakerDiagnostic(const Tab5SpeakerDiagnostic&) = delete;
  Tab5SpeakerDiagnostic& operator=(const Tab5SpeakerDiagnostic&) = delete;
  Tab5SpeakerState state() const { return state_; }

  // Requests the amplifier off (verified by readback) and then opens the ES8388
  // muted at `volume`. Nothing is audible before this returns, and no tone
  // follows by itself. The interactive mode passes its own audible percentage;
  // the speaker-only probe keeps kTab5SpeakerVolume.
  bool prepare(uint8_t volume = kTab5SpeakerVolume) {
    if (prepared_ || used_ || volume > 100) return false;
    used_ = true;  // a failed initialization also consumes the only attempt
    const auto start = io_.now_ms();
    // Request low before changing output mode/direction; never reset the chip.
    if (!off() || elapsed(start) > kTab5SpeakerDacMs) return fail();
    const auto open_start = io_.now_ms();
    if (!io_.open_dac(volume, true, kTab5SpeakerDacMs) ||
        elapsed(open_start) > kTab5SpeakerDacMs || elapsed(start) > 2 * kTab5SpeakerDacMs)
      return fail();
    prepared_ = true;
    used_ = false;
    state_ = Tab5SpeakerState::Ready;
    return true;
  }

  // Leaves the state after a refused or absent start: amplifier off, verified,
  // DAC closed. No tone is written and nothing is enabled.
  bool disarm() {
    if (!prepared_ || used_) return false;
    used_ = true;
    if (!off()) return fail();
    const auto close_start = io_.now_ms();
    if (!io_.close_dac(kTab5SpeakerDacMs) || elapsed(close_start) > kTab5SpeakerDacMs)
      return fail();
    state_ = Tab5SpeakerState::SafeOff;
    return true;
  }

  // Plays the one tone, then drops the amplifier, mutes and closes the DAC.
  // Only `explicit_action` (a deliberate start) can produce a tone. `tone_peak`
  // is the amplitude of the shared 1 kHz/250 ms shape: the speaker-only probe
  // keeps the quiet kTab5SpeakerTonePeak, and the interactive mode passes its own
  // audible level.
  bool play_once(bool explicit_action, int16_t tone_peak = kTab5SpeakerTonePeak) {
    if (!explicit_action || !prepared_ || used_ || tone_peak <= 0) return false;
    used_ = true;
    const auto start = io_.now_ms();
    if (!on() || elapsed(start) > kTab5SpeakerDacMs) return fail();
    const auto unmute_start = io_.now_ms();
    if (!io_.set_mute(false, kTab5SpeakerIoMs) || elapsed(unmute_start) > kTab5SpeakerIoMs)
      return fail();
    // One generated block, kept off the (small) main task stack.
    int16_t* pcm = tone_buffer();
    const size_t frames = tab5_speaker_tone(pcm, kTab5SpeakerToneFrames, tone_peak);
    if (!frames) return fail();
    state_ = Tab5SpeakerState::Playing;
    const auto tone_start = io_.now_ms();
    // Bounded by the adapter's own I2S wait; success is queued audio, not proof
    // that the amplifier or the speaker reproduced it.
    if (!io_.tone(pcm, frames, kTab5SpeakerToneWriteMs) ||
        elapsed(tone_start) > kTab5SpeakerToneWriteMs)
      return fail();
    // Mute first: a failed amplifier write must not leave an audible path.
    const auto mute_start = io_.now_ms();
    if (!io_.set_mute(true, kTab5SpeakerIoMs) || elapsed(mute_start) > kTab5SpeakerIoMs)
      return fail();
    // The verification step: only acknowledged, read-back register values let
    // this report SafeOff. A timer can never prove the hardware is quiet.
    if (!off()) return fail();
    const auto close_start = io_.now_ms();
    if (!io_.close_dac(kTab5SpeakerDacMs) || elapsed(close_start) > kTab5SpeakerDacMs)
      return fail();
    state_ = Tab5SpeakerState::SafeOff;
    return true;
  }

  // The polarity probe, kept for hardware whose enable convention is still
  // unknown: two tones inside one bounded sequence, first with the amplifier
  // control driven HIGH, then driven LOW, each preceded by the reviewed off state
  // and followed by it again. Nothing calls this automatically - the Tab5 audio
  // UI measured it and now plays only the confirmed HIGH convention - but the
  // inverted convention stays expressible for another board or a second look.
  // `hook` (optional) is called between that attempt's readback and its tone, so
  // a caller can say which one is playing. The end state is mute requested, the
  // amplifier requested off in the reviewed sense and read back, and the DAC
  // closed: if the other polarity is the real enable, the muted and released DAC
  // leaves nothing to amplify, so no attempt can end with an audible path left
  // open. Only
  // `explicit_action` (a deliberate tap) can produce sound. Returns false and
  // leaves the state sticky Unknown on any failure, and only one probe runs per
  // instance.
  bool play_probe(bool explicit_action, int16_t tone_peak, Tab5SpeakerAttemptHook hook,
                  void* ctx) {
    if (!explicit_action || !prepared_ || used_ || tone_peak <= 0) return false;
    used_ = true;
    const auto start = io_.now_ms();
    if (!off() || elapsed(start) > kTab5SpeakerDacMs) return fail();
    int16_t* pcm = tone_buffer();
    const size_t frames = tab5_speaker_tone(pcm, kTab5SpeakerToneFrames, tone_peak);
    if (!frames) return fail();
    constexpr Tab5SpeakerPa kDrives[kTab5SpeakerProbeAttempts] = {Tab5SpeakerPa::High,
                                                                  Tab5SpeakerPa::Low};
    for (const Tab5SpeakerPa drive : kDrives) {
      // The one bit that differs between the attempts, written and read back
      // before the label is shown so the screen never names an attempt whose
      // control value did not reach the expander.
      if (!set_drive(drive)) return fail();
      if (hook) hook(ctx, drive);
      const auto unmute_start = io_.now_ms();
      if (!io_.set_mute(false, kTab5SpeakerIoMs) || elapsed(unmute_start) > kTab5SpeakerIoMs)
        return fail();
      state_ = Tab5SpeakerState::Playing;
      const auto tone_start = io_.now_ms();
      // Success is queued audio, not proof that the amplifier or the speaker
      // reproduced it: only the operator's ears can answer that.
      if (!io_.tone(pcm, frames, kTab5SpeakerToneWriteMs) ||
          elapsed(tone_start) > kTab5SpeakerToneWriteMs)
        return fail();
      // Mute before the next step: a failed control write must not leave an
      // audible path.
      const auto mute_start = io_.now_ms();
      if (!io_.set_mute(true, kTab5SpeakerIoMs) || elapsed(mute_start) > kTab5SpeakerIoMs)
        return fail();
      // Back to the reviewed off state between the attempts and after the last
      // one, verified by readback rather than assumed from the timer.
      if (!off()) return fail();
    }
    const auto close_start = io_.now_ms();
    if (!io_.close_dac(kTab5SpeakerDacMs) || elapsed(close_start) > kTab5SpeakerDacMs)
      return fail();
    state_ = Tab5SpeakerState::SafeOff;
    return true;
  }

 private:
  uint64_t elapsed(uint64_t start) const { return io_.now_ms() - start; }
  // One generated block, shared by both players and off the (small) main task
  // stack. This class is documented non-reentrant: one attempt at a time.
  static int16_t* tone_buffer() {
    static int16_t pcm[kTab5SpeakerToneFrames];
    return pcm;
  }
  // Masked read-modify-write with a real readback: read the byte, change only
  // pin 1, write it, then read it again and compare. A failed or late write may
  // already have applied, so nothing is inferred from the acknowledgement.
  bool update(uint8_t reg, uint8_t bit) {
    uint8_t before = 0, after = 0;
    auto start = io_.now_ms();
    if (!io_.read(reg, before, kTab5SpeakerIoMs) || elapsed(start) > kTab5SpeakerIoMs)
      return false;
    const auto value = static_cast<uint8_t>((before & ~kTab5SpeakerMask) | (bit & kTab5SpeakerMask));
    start = io_.now_ms();
    if (!io_.write(reg, value, kTab5SpeakerIoMs) || elapsed(start) > kTab5SpeakerIoMs)
      return false;
    start = io_.now_ms();
    return io_.read(reg, after, kTab5SpeakerIoMs) && elapsed(start) <= kTab5SpeakerIoMs &&
           after == value;
  }
  // Same registers, order and masks as tab5_prepare_speaker_off (panel_tab5.hpp):
  // the documented speaker-off state, here with readback verification. Never
  // make a possibly-high latch an output, and never reset the expander.
  bool off() {
    return update(kTab5SpeakerConfigReg, 0) && update(kTab5SpeakerDirectionReg, 0) &&
           update(kTab5SpeakerLevelReg, kTab5SpeakerMask);
  }
  // The single enable step for the confirmed convention: pin 1's control bit in
  // 0x05, which the reviewed off sequence cleared (tab5_prepare_speaker_off).
  // 0x03 is left untouched, so off() reverts one register and re-verifies the
  // whole state. Register 0x05 reads as the pin's control/polarity bit on this
  // expander, and HIGH is the polarity the physical amplifier honoured when both
  // were played by ear.
  bool on() { return set_drive(kTab5SpeakerConfirmedDrive); }
  // The only place the control's level is decided: one bit of 0x05, every other
  // bit preserved from the read, the written value read back and compared.
  bool set_drive(Tab5SpeakerPa pa) {
    return update(kTab5SpeakerConfigReg, pa == Tab5SpeakerPa::High ? kTab5SpeakerMask : uint8_t{0});
  }
  bool fail() {
    used_ = true;
    state_ = Tab5SpeakerState::Unknown;
    // Best effort only; never upgrades Unknown to a physical-off claim.
    io_.set_mute(true, kTab5SpeakerIoMs);
    off();
    io_.close_dac(kTab5SpeakerDacMs);
    return false;
  }
  IO& io_;
  Tab5SpeakerState state_ = Tab5SpeakerState::Unknown;
  bool prepared_ = false;
  bool used_ = false;
};

// The ESP adapter: buses and codec, the deliberate start, the single attempt,
// then the result line and teardown. Never returns, so a repeat needs a boot.
[[noreturn]] void tab5_speaker_diagnostic_run();

}  // namespace hgp
