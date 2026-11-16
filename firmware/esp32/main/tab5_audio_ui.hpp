// Interactive Tab5 audio diagnostic: the pure screen/state half, host-testable.
//
// No ESP-IDF, I2S, display or codec dependency lives here: tab5_audio_ui.cpp
// brings up the panel, touch, I2S and both codecs, and this header owns only
// what can be tested on the host. It reuses, and never reimplements:
//   - the confirmed preview model and its touch feedback
//     (tab5_diagnostic_preview / tab5_diagnostic_touch),
//   - the microphone level math and its bounded windows (tab5_mic_*),
//   - the speaker state machine with masked expander read-modify-write and
//     sticky Unknown (tab5_speaker_*).
//
// The mode is deliberately narrow: every tap on the top half plays ONE 1 kHz,
// 250 ms tone at about -12 dBFS through the reviewed speaker state machine, with
// the amplifier control driven HIGH - the enable convention measured on this
// unit, where HIGH was audible and LOW was silent - and the reviewed off state
// requested and read back immediately after that tone. Taps inside a short
// cooldown are refused instead of queued, no tone starts while a microphone
// window owns the ADC, and nothing plays at boot. The bottom half keeps the
// bounded ES7210 windows of the microphone probe. Nothing here or in the adapter
// stores, streams or logs samples, and no App, NVS, identity, network, OTA or
// host seed exchange exists in this mode.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

#include "hg/ui.hpp"
#include "tab5_diagnostic.hpp"
#include "tab5_mic_diagnostic.hpp"
#include "tab5_speaker_diagnostic.hpp"

namespace hgp {

// The two labelled halves of the panel. The tone keeps the important half:
// exactly on the split belongs to the microphone, so the tone is never reached
// by a stray tap in the middle of the screen.
enum class Tab5AudioTarget : uint8_t { None, Tone, Mic };

inline Tab5AudioTarget tab5_audio_target_at(int x, int y, int width, int height) {
  if (width <= 0 || height <= 0) return Tab5AudioTarget::None;
  if (x < 0 || y < 0 || x >= width || y >= height) return Tab5AudioTarget::None;
  return y < height / 2 ? Tab5AudioTarget::Tone : Tab5AudioTarget::Mic;
}

// The interactive mode plays a tone at a level the operator can actually hear:
// about -12 dBFS, still well below full scale, through the same 1 kHz, 250 ms
// shape the speaker-only probe uses. Its own percentage is higher than that
// probe's conservative level because the first physical run of that level
// produced no audible sound.
// 32768 * 10^(-12/20) = 8212, so 8190 is about -12.05 dBFS with headroom.
inline constexpr int16_t kTab5AudioTonePeak = 8190;
// esp_codec_dev_set_out_vol takes a percentage: 75 is loud enough to be judged
// by ear and still leaves the codec's own headroom above it.
inline constexpr uint8_t kTab5AudioVolume = 75;
static_assert(kTab5AudioTonePeak > 0 && kTab5AudioTonePeak < 32767,
              "the interactive tone must never clip");
static_assert(kTab5AudioVolume > 0 && kTab5AudioVolume <= 100,
              "esp_codec_dev volume is a percentage");

// Which way the amplifier control (expander 0x43 pin 1) is driven. HIGH is the
// convention measured on this unit: with the control HIGH the operator heard the
// tone, with it LOW the amplifier stayed silent. The inverted level is still
// expressible - the reviewed state machine can write either bit and the retained
// play_probe() still drives both ways - but nothing plays it automatically any
// more: the default path is the confirmed one.
using Tab5AudioPa = Tab5SpeakerPa;
inline constexpr Tab5AudioPa kTab5AudioConfirmedPa = kTab5SpeakerConfirmedDrive;
inline constexpr Tab5AudioPa kTab5AudioInvertedPa = Tab5SpeakerPa::Low;

inline const char* tab5_audio_pa_text(Tab5AudioPa pa) {
  return pa == Tab5AudioPa::High ? "pin HIGH" : "pin LOW";
}

// How long each state's status line is held on screen before the next one may
// replace it: the A/B labels used to flash past faster than they could be read.
inline constexpr uint32_t kTab5AudioStatusHoldMs = 1500;
// One tap at a time: a tap inside this window after the previous tone started is
// refused, not queued, so a burst of taps can never stack tones on the amplifier.
inline constexpr uint32_t kTab5AudioToneCooldownMs = 1000;
static_assert(kTab5AudioToneCooldownMs > kTab5SpeakerToneMs,
              "the cooldown must outlast the tone it follows");
static_assert(kTab5AudioStatusHoldMs >= kTab5AudioToneCooldownMs,
              "a status line must stay readable for at least a tap's cycle");

// True when a tone may start again after `last_ms`: the cooldown must have fully
// passed, and a backwards clock proves no interval at all, so it refuses too.
inline bool tab5_audio_tone_ready(uint64_t last_ms, uint64_t now_ms) {
  return now_ms >= last_ms && now_ms - last_ms >= kTab5AudioToneCooldownMs;
}

// The states the status line can name, in the order a tap goes through them:
// one tone playing, then its verified off state or the honest unknown, then back
// to ready for the next tap.
enum class Tab5AudioTonePhase : uint8_t { Ready, Playing, Verified, Unknown };

// "TONE: playing (amplifier on)" / "TONE: off and verified" / "TONE: unknown" /
// "TONE: ready": the held status line. Complete or refused (0), never truncated
// into a different state.
inline size_t tab5_audio_status_line(char* out, size_t capacity, Tab5AudioTonePhase phase) {
  if (!out || capacity == 0) return 0;
  const char* text = "TONE: ready";
  switch (phase) {
    case Tab5AudioTonePhase::Playing: text = "TONE: playing (amplifier on)"; break;
    case Tab5AudioTonePhase::Verified: text = "TONE: off and verified"; break;
    case Tab5AudioTonePhase::Unknown: text = "TONE: unknown"; break;
    case Tab5AudioTonePhase::Ready: break;
  }
  const int written = std::snprintf(out, capacity, "%s", text);
  if (written < 0 || size_t(written) >= capacity) return 0;
  return size_t(written);
}

// "TAP TOP: one 250 ms tone   TAP BOTTOM: mic level": the persistent hint, in
// the panel's own line and in the bottom bar.
inline size_t tab5_audio_hint_line(char* out, size_t capacity) {
  if (!out || capacity == 0) return 0;
  const int written = std::snprintf(out, capacity, "TAP TOP: one %u ms tone   TAP BOTTOM: mic level",
                                    unsigned(kTab5SpeakerToneMs));
  if (written < 0 || size_t(written) >= capacity) return 0;
  return size_t(written);
}

// The bottom bar's shorter copy of the same hint.
inline size_t tab5_audio_bar_hint(char* out, size_t capacity) {
  if (!out || capacity == 0) return 0;
  const int written = std::snprintf(out, capacity, "TAP TOP: tone   TAP BOTTOM: mic level");
  if (written < 0 || size_t(written) >= capacity) return 0;
  return size_t(written);
}

// "TONES PLAYED: 3" before any outcome exists, and "TONES PLAYED: 3   LAST:
// verified" / "LAST: UNKNOWN" afterwards: the running counter of tones this boot
// played, so a tester can tap repeatedly and see that each tap registered.
inline size_t tab5_audio_counter_line(char* out, size_t capacity, unsigned tones,
                                     Tab5AudioTonePhase last) {
  if (!out || capacity == 0) return 0;
  int written = 0;
  if (tones > 0 && last == Tab5AudioTonePhase::Verified) {
    written = std::snprintf(out, capacity, "TONES PLAYED: %u   LAST: verified", tones);
  } else if (tones > 0 && last == Tab5AudioTonePhase::Unknown) {
    written = std::snprintf(out, capacity, "TONES PLAYED: %u   LAST: UNKNOWN", tones);
  } else {
    written = std::snprintf(out, capacity, "TONES PLAYED: %u", tones);
  }
  if (written < 0 || size_t(written) >= capacity) return 0;
  return size_t(written);
}

// What the adapter must do for one touch contact. A contact decides once, when
// it starts: moves and the lift only refresh the coordinate line.
enum class Tab5AudioAction : uint8_t { None, PlayTone, MeasureMic };

// A few seconds, well inside kTab5MicSessionMs: the standalone probe's 30 s
// session is far longer than a level check needs, and this window owns the ADC
// for its whole length. The adapter closes the ES7210 at the bound and keeps no
// samples at all.
inline constexpr uint32_t kTab5AudioMicWindowMs = 4000;
// Live level updates follow the standalone probe's cadence through
// tab5_mic_report_due, so the two read the same way on screen and on serial.

inline bool tab5_audio_mic_window_open(uint64_t start_ms, uint64_t now_ms) {
  // A backwards clock closes the window instead of holding the ADC open.
  return now_ms >= start_ms && now_ms - start_ms < kTab5AudioMicWindowMs;
}

// "MIC: 33%": the last measured level, or "MIC: --" for none. Negative means
// nothing was measured yet; above 100 is clamped, never printed as impossible.
inline size_t tab5_audio_mic_line(char* out, size_t capacity, int percent) {
  if (!out || capacity == 0) return 0;
  const int written = percent < 0
                          ? std::snprintf(out, capacity, "MIC: --")
                          : std::snprintf(out, capacity, "MIC: %d%%", percent > 100 ? 100 : percent);
  if (written < 0 || size_t(written) >= capacity) return 0;
  return size_t(written);
}

// The window's result: the loudest block's level, in the same percentage the
// App's VAD (hg::level_percent) and the standalone probe's "mic 33%" lines use,
// so the numbers stay comparable. -1 when nothing was measured.
inline int tab5_audio_mic_result(const Tab5MicStats& stats) {
  if (!stats.blocks) return -1;
  return int(hg::level_percent(stats.block_rms_peak));
}

// The screen: a Hermes-style preview model from tab5_diagnostic_preview(), with
// the held tone status as its headline, the microphone level, the persistent
// hint and the running tone counter in its detail lines, and the confirmed
// coordinate feedback in its bottom bar. Pure: it never opens a device, and
// every transition the adapter must honour is decided here.
class Tab5AudioScreen {
 public:
  Tab5AudioScreen(int width, int height) : width_(width), height_(height) {
    model_.title = "Hermes - TAB5 AUDIO";
    char bar[48];
    model_.hint = tab5_audio_bar_hint(bar, sizeof(bar)) ? bar : "TAP TOP: tone";
    refresh();
  }

  const hg::UiModel& model() const { return model_; }

  // One touch contact: down decides, a move or lift only refreshes feedback.
  // Returns what the adapter must do, and never the same action twice for one
  // contact. While the microphone window runs nothing else is started, so the
  // amplifier is never enabled mid-capture.
  Tab5AudioAction touch(bool touching, int x, int y, uint64_t now_ms) {
    tab5_diagnostic_touch(model_, touching, x, y);
    if (!touching) {
      finger_ = false;
      refresh();
      return Tab5AudioAction::None;
    }
    const bool decided = !finger_;
    finger_ = true;
    refresh();
    if (!decided || !fault_.empty()) return Tab5AudioAction::None;
    switch (tab5_audio_target_at(x, y, width_, height_)) {
      case Tab5AudioTarget::Tone:
        // No tone while the ADC is captured or inside the cooldown, so a burst
        // of taps can neither queue tones nor overlap two of them.
        if (mic_active_) return Tab5AudioAction::None;
        if (tapped_ && !tab5_audio_tone_ready(last_tone_ms_, now_ms)) return Tab5AudioAction::None;
        tapped_ = true;
        last_tone_ms_ = now_ms;
        ++tones_;
        result_pending_ = false;
        enter(Tab5AudioTonePhase::Playing, now_ms);
        refresh();
        return Tab5AudioAction::PlayTone;
      case Tab5AudioTarget::Mic:
        if (!mic_fault_.empty() || mic_active_) return Tab5AudioAction::None;
        mic_active_ = true;
        mic_percent_ = -1;  // a new window measures its own level
        refresh();
        return Tab5AudioAction::MeasureMic;
      case Tab5AudioTarget::None: break;
    }
    return Tab5AudioAction::None;
  }

  // The adapter's own result for the tone it just played. Unknown is sticky: a
  // failed off request can never be shown as a verified one.
  void tone_done(Tab5SpeakerState state, uint64_t now_ms) {
    last_result_ =
        state == Tab5SpeakerState::SafeOff ? Tab5AudioTonePhase::Verified : Tab5AudioTonePhase::Unknown;
    result_pending_ = true;
    advance(now_ms);
    refresh();
  }

  // Lets the held status line move on once its minimum hold has passed. Returns
  // true when the visible state changed, so the adapter redraws exactly then.
  bool tick(uint64_t now_ms) {
    if (!advance(now_ms)) return false;
    refresh();
    return true;
  }

  Tab5AudioTonePhase tone_phase() const { return phase_; }
  // The last outcome the state machine reported, or Ready before the first tone.
  Tab5AudioTonePhase last_result() const { return last_result_; }
  // Tones this boot started, one per accepted tap: the line the tester watches.
  unsigned tones_played() const { return tones_; }

  bool mic_active() const { return mic_active_; }
  // Live level of the open window; false when no window is open.
  bool mic_level(uint8_t percent) {
    if (!mic_active_) return false;
    mic_percent_ = percent > 100 ? 100 : int(percent);
    refresh();
    return true;
  }
  // The window ended: the ADC is closed and the loudest block is the result.
  void mic_done(const Tab5MicStats& stats) {
    mic_active_ = false;
    mic_percent_ = tab5_audio_mic_result(stats);
    refresh();
  }
  // The ES7210 did not open: said on screen, and not retried into an unknown
  // codec state (the standalone probe is one attempt per boot for the same
  // reason). The tone half stays available.
  void set_mic_fault(std::string reason) {
    mic_active_ = false;
    mic_percent_ = -1;
    mic_fault_ = std::move(reason);
    refresh();
  }
  // The shared audio path never started: both halves are refused, and the
  // reason stays on screen.
  void set_audio_fault(std::string reason) {
    mic_active_ = false;
    fault_ = "AUDIO: " + std::move(reason);
    refresh();
  }
  // No touch input at all: this mode is entirely touch-driven, so both halves
  // are refused and the reason stays on screen.
  void set_input_fault(std::string reason) {
    mic_active_ = false;
    fault_ = "INPUT: " + std::move(reason);
    refresh();
  }

 private:
  // A hold releases on a backwards clock instead of freezing a line for ever.
  static bool hold_elapsed(uint64_t start, uint64_t now, uint32_t ms) {
    return now < start || now - start >= ms;
  }
  void enter(Tab5AudioTonePhase phase, uint64_t now_ms) {
    phase_ = phase;
    hold_start_ = now_ms;
  }
  // One step of the held status line: the result replaces "playing" only after
  // the play line was readable, and it is itself held before "ready" returns.
  bool advance(uint64_t now_ms) {
    if (result_pending_) {
      if (!hold_elapsed(hold_start_, now_ms, kTab5AudioStatusHoldMs)) return false;
      result_pending_ = false;
      enter(last_result_, now_ms);
      return true;
    }
    if ((phase_ == Tab5AudioTonePhase::Verified || phase_ == Tab5AudioTonePhase::Unknown) &&
        hold_elapsed(hold_start_, now_ms, kTab5AudioStatusHoldMs)) {
      enter(Tab5AudioTonePhase::Ready, now_ms);
      return true;
    }
    return false;
  }

  void refresh() {
    model_.screen = mic_active_ ? hg::Screen::Listening : hg::Screen::Offline;
    model_.level = mic_active_ && mic_percent_ > 0 ? static_cast<uint8_t>(mic_percent_) : 0;
    char line[48];
    model_.headline =
        tab5_audio_status_line(line, sizeof(line), phase_) ? line : "TONE: ?";
    std::string detail;
    if (!fault_.empty()) {
      detail = fault_;
    } else if (!mic_fault_.empty()) {
      detail = "MIC: " + mic_fault_;
    } else if (mic_active_ && mic_percent_ < 0) {
      detail = "MIC: listening";
    } else {
      char mic[24];
      detail = tab5_audio_mic_line(mic, sizeof(mic), mic_percent_) ? mic : "MIC: --";
    }
    char hint[64];
    if (tab5_audio_hint_line(hint, sizeof(hint))) detail += std::string("\n") + hint;
    char counter[64];
    if (tab5_audio_counter_line(counter, sizeof(counter), tones_, last_result_))
      detail += std::string("\n") + counter;
    // The sticky warning stays while the last outcome is unknown: a failed off
    // request must never let the screen be read as "the amplifier is off".
    const bool unknown = last_result_ == Tab5AudioTonePhase::Unknown;
    if (fault_.empty() && tones_ > 0 && unknown)
      detail += "\nTONE: amplifier state unknown - do not assume it is off";
    model_.detail = detail;
    model_.caption_lines = unknown ? 4 : 3;
  }

  hg::UiModel model_ = tab5_diagnostic_preview();
  int width_ = 0, height_ = 0;
  Tab5AudioTonePhase phase_ = Tab5AudioTonePhase::Ready;
  Tab5AudioTonePhase last_result_ = Tab5AudioTonePhase::Ready;
  bool result_pending_ = false;
  uint64_t hold_start_ = 0;    // when the shown state became current
  uint64_t last_tone_ms_ = 0;  // when the last accepted tone tap started
  bool tapped_ = false;        // a tone tap was accepted this boot
  unsigned tones_ = 0;         // accepted tone taps this boot
  bool mic_active_ = false;
  int mic_percent_ = -1;  // -1: nothing measured yet
  bool finger_ = false;   // a contact already decided
  std::string mic_fault_, fault_;  // fault_: the whole first line, e.g. "AUDIO: ..."
};

// The adapter: display, touch, I2S and both codecs, the two labelled halves, the
// bounded microphone window and the repeatable single-tone path. Never returns,
// so a reboot is needed to clear the screen state. Implemented in
// tab5_audio_ui.cpp, which also reuses main.cpp's display/touch gate so the panel
// comes up exactly as the confirmed preview does.
[[noreturn]] void tab5_audio_ui_run();

}  // namespace hgp
