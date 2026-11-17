// Original ILI9881C/GT911 Tab5 only. Hardware facts from Espressif's
// m5stack_tab5 1.3.1, esp-bsp 73ee07b1ad56f865f13a2739be8bb6808c1da58a.
#pragma once
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string_view>

#include "battery_curve.hpp"
#include "board.hpp"

namespace hgp {
inline BoardConfig tab5_config(const char* name) {
  BoardConfig b{};
  b.name = name;
  // The radio is on: the C6 rides SDIO slot 1 with the pin map in this board's
  // sdkconfig.defaults, and only the normal startup path starts ESP-Hosted
  // (main.cpp), so the display/audio diagnostics never touch the C6.
  b.radio_enabled = true;
  b.i2c = {31, 32, 400000};
  b.lcd.enabled = true;
  b.lcd.width = 720;
  b.lcd.height = 1280;
  b.lcd.swap_xy = b.lcd.mirror_x = b.lcd.mirror_y = b.lcd.invert = false;
  b.lcd.backlight = 22;
  b.lcd.font_scale = 2;
  b.lcd.bus.type = LcdBus::Type::Dsi;
  auto& d = b.lcd.dsi;
  d.controller = DsiController::Ili9881c;
  d.vdd_io_ldo4 = false;
  d.phy_clk_auto = true;
  d.lane_bit_rate_mbps = 1000;
  d.dpi_clock_mhz = 60;
  d.hsync_pulse_width = 40;
  d.hsync_back_porch = 140;
  d.hsync_front_porch = 40;
  d.vsync_pulse_width = 4;
  d.vsync_back_porch = 20;
  d.vsync_front_porch = 20;
  b.touch.enabled = true;
  b.touch.controller = TouchController::Gt911;
  b.touch.addr = 0x14;
  b.touch.width = 720;
  b.touch.height = 1280;
  b.touch.ppi = 294;  // diagonal pixel count / nominal 5 inches
  // After the GT911 probe, GPIO23 is held low; the managed driver must not own INT.
  b.touch.int_gpio = -1;
  // Audio: the ES8388 DAC (speaker) and the ES7210 ADC (microphones) hang off
  // one duplex I2S_NUM_1 bus, and the ES8388's amplifier is enabled by the
  // 0x43 expander's pin 1 instead of a GPIO (BSP_POWER_AMP_IO is GPIO_NUM_NC).
  // 24 kHz mono 16-bit, the rate and width every SDK codec board runs; the
  // board_pa hook in main.cpp drives that expander bit (Tab5Amplifier).
  b.codec.enabled = true;
  b.codec.mclk = 30;  // BSP_I2S_MCLK
  b.codec.bclk = 27;  // BSP_I2S_SCLK
  b.codec.ws = 29;    // BSP_I2S_LCLK
  b.codec.dout = 26;  // BSP_I2S_DOUT (ES8388 SDI)
  b.codec.din = 28;   // BSP_I2S_DSIN (ES7210 SDOUT)
  b.codec.pa = -1;    // never a GPIO: the amplifier is the expander's pin 1
  b.codec.speaker_pa = false;
  b.codec.speaker = SpeakerCodec::Es8388;  // DAC only; its own ADC is unused
  b.codec.mic = MicCodec::Es7210;
  b.codec.es7210_mics = 0x03;   // MIC1|MIC2: what the working microphone probe opened
  b.codec.mic_gain_db = 24.0f;  // the SDK default, the gain those probes captured with
  b.codec.stereo32 = false;
  b.codec.rmnm_mics = false;
  // Measured on this unit: a -40 dBFS tone at volume 40 was silent and the same
  // tone at 75 was audible. The app's stored setting overrides this at startup.
  b.codec.volume = 75;
  // Clock and motion, both on the shared bus this profile already probes
  // (0x10 ES8388, 0x14 GT911, 0x43/0x44 PI4IO expanders). This unit's own I2C
  // scan (console `diag`) put the RX8130CE at 0x32 and the BMI270 at 0x68.
  //
  // The RX8130CE holds UTC while its cell has charge. port_time.cpp already
  // builds that driver for RtcChip::Rx8130 and tolerates an absent or
  // unanswered chip (it logs and leaves the clock to network time), so nothing
  // here can keep the gadget from booting without it.
  b.rtc.enabled = true;
  b.rtc.chip = RtcChip::Rx8130;
  b.rtc.addr = 0x32;
  // The BMI270 gives face-down sleep through the existing motion driver, which
  // starts the accelerometer only (no gyroscope). The mounting orientation of
  // this chip is unmeasured, so the axes stay the chip's own: a guessed remap
  // would make face-down sleep trigger in the wrong position, and the identity
  // map only risks that the sleep fires on the opposite face. Verify with `diag`
  // (`motion` while lying screen up and face down) before trusting it.
  b.imu.enabled = true;
  b.imu.chip = ImuChip::Bmi270;
  b.imu.addr = 0x68;
  // Measured on this unit: lying screen up, the raw chip z reads about -989 mg,
  // and the face-down rule expects a negative z, so the chip's z is inverted
  // here. With this sign, screen up is positive and face down is about -989.
  b.imu.z_sign = -1;
  return b;
}

// BSP_POWER_AMP_IO is GPIO_NUM_NC on this board: the amplifier enable is the
// 0x43 expander's pin 1 (see tab5_prepare_speaker_off and Tab5Amplifier).

// Microphone path only, from the same BSP (bsp_audio.c): the ES7210 ADC on the
// shared I2S bus, RX only. These clocks also drive the ES8388, so a mode that
// uses this map must never open the DAC, the amplifier or the expander.
struct Tab5MicPins {
  int mclk, bclk, ws, din;
};

inline constexpr Tab5MicPins tab5_mic_pins() {
  // BSP_I2S_MCLK/SCLK/LCLK/DSIN. BSP_I2S_DOUT (GPIO 26) is deliberately absent:
  // the diagnostic never transmits.
  return Tab5MicPins{30, 27, 29, 28};
}

// Speaker path only, from the same BSP (bsp_audio.c): the ES8388 DAC as a TX
// client on the shared I2S bus. The clocks also drive the ES7210, so a mode
// that uses this map must never open the ADC, and BSP_I2S_DSIN (GPIO 28) stays
// an input nobody reads.
struct Tab5SpeakerPins {
  int mclk, bclk, ws, dout;
};

inline constexpr Tab5SpeakerPins tab5_speaker_pins() {
  // BSP_I2S_MCLK/SCLK/LCLK/DOUT. The microphone input is not part of this map.
  return Tab5SpeakerPins{30, 27, 29, 26};
}

// The ESP32-C6 sits behind its own power gate: a second PI4IOE5V6408 at 0x44,
// pin 0 (BSP_WIFI_EN), active high. Drive the latch high before turning the pin
// into an output so enabling it cannot glitch the C6 supply low. Only pin 0 is
// changed; requires the same exclusive ownership as the other expander writes.
template<class Read, class Write>
bool tab5_enable_wifi_power(Read read, Write write) {
  auto update = [&](uint8_t reg, uint8_t value) {
    uint8_t current = 0;
    if (!read(reg, current)) return false;
    return write(reg, static_cast<uint8_t>((current & ~0x01u) | value));
  };
  return update(0x05, 0x01) && update(0x07, 0x00) && update(0x03, 0x01);
}

// Prepare the PA control as a low push-pull output before audio starts.
// Requires exclusive ownership across every read/write, with no cached BSP
// expander owner. Only pin 1 is changed relative to each read. A failed I2C
// transaction leaves its state unknown; an already-high pin stays high until
// the low latch write takes effect.
template<class Read, class Write>
bool tab5_prepare_speaker_off(Read read, Write write) {
  auto update = [&](uint8_t reg, uint8_t value) {
    uint8_t current = 0;
    if (!read(reg, current)) return false;
    return write(reg, static_cast<uint8_t>((current & ~0x02u) | value));
  };
  return update(0x05, 0) && update(0x07, 0) && update(0x03, 0x02);
}

// The amplifier enable on that expander. Same chip address, pin, registers and
// polarity as the audible A/B probe (tab5_speaker_diagnostic.hpp): register
// 0x05 bit 1 driven HIGH is when the operator heard the tone, LOW when the
// speaker stayed silent. Only that one bit is ever changed, relative to each
// read, and every write is read back and compared.
inline constexpr uint8_t kTab5PaExpanderAddr = 0x43;
inline constexpr uint8_t kTab5PaPin = 1;
inline constexpr uint8_t kTab5PaMask = static_cast<uint8_t>(1u << kTab5PaPin);  // 0x02
inline constexpr uint8_t kTab5PaConfigReg = 0x05;     // the enable's level bit
inline constexpr uint8_t kTab5PaDirectionReg = 0x07;  // bit 1: 0 = driven output
inline constexpr uint8_t kTab5PaLevelReg = 0x03;      // bit 1: off level
inline constexpr uint32_t kTab5PaIoMs = 50;           // one expander read or write

// Logged whenever a request could not be verified. It never says the amplifier
// is off: a failed read or write leaves the hardware state unknown.
inline constexpr const char* kTab5PaFailureLog =
    "Tab5 amplifier: expander 0x43 pin 1 write or readback failed; the codec is muted and the "
    "amplifier stays disabled until reboot (its state is unknown, not off)";

// The board_pa hook CodecSpeaker calls has no way to report an error, so the
// failure has to live here: a failed read, write or readback latches for good,
// mutes the codec through IO::mute() and refuses every later request. Refusing
// to enable again is the point: a write that reported an error may still have
// reached the chip, so an amplifier that could not be verified is never turned
// on a second time, and an unverified off is never reported as off.
//
// IO provides bool read(reg, value, timeout_ms), bool
// write(reg, value, timeout_ms) on the 0x43 expander only (no BSP expander, no
// reset) and void mute(), which silences the codec.
template <class IO>
class Tab5Amplifier {
 public:
  explicit Tab5Amplifier(IO& io) : io_(io) {}
  Tab5Amplifier(const Tab5Amplifier&) = delete;
  Tab5Amplifier& operator=(const Tab5Amplifier&) = delete;

  // The state audio starts from: the reviewed off sequence of
  // tab5_prepare_speaker_off, each step read back. False latches the failure.
  bool prepare_off() {
    if (failed_) return false;
    return update(kTab5PaConfigReg, 0) && update(kTab5PaDirectionReg, 0) &&
           update(kTab5PaLevelReg, kTab5PaMask);
  }

  // HIGH enables and LOW disables the measured convention. True only when the
  // written value was read back; the caller may then believe the state.
  bool set(bool on) {
    if (failed_) return false;
    return update(kTab5PaConfigReg, on ? kTab5PaMask : uint8_t{0});
  }

  // True after any unverified request; the amplifier is never enabled again.
  bool failed() const { return failed_; }

 private:
  // One masked read-modify-write with a real readback: read the byte, change
  // only pin 1, write it, read it again and compare. Nothing is inferred from
  // the write's acknowledgement.
  bool update(uint8_t reg, uint8_t bit) {
    if (failed_) return false;
    uint8_t before = 0, after = 0;
    if (!io_.read(reg, before, kTab5PaIoMs)) return fail();
    const auto value = static_cast<uint8_t>((before & ~kTab5PaMask) | (bit & kTab5PaMask));
    if (!io_.write(reg, value, kTab5PaIoMs)) return fail();
    if (!io_.read(reg, after, kTab5PaIoMs) || after != value) return fail();
    return true;
  }
  bool fail() {
    failed_ = true;
    io_.mute();  // best effort: no audible path is left unrequested
    return false;
  }
  IO& io_;
  bool failed_ = false;
};

// Callbacks operate on the existing shared-bus device, never reset the chip.
template<class Read, class Write, class Verify>
bool tab5_enable_display_touch(Read read, Write write, Verify verify) {
  struct Change { uint8_t reg, mask, old; } changes[5];
  unsigned count = 0;
  auto update = [&](uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t old = 0;
    if (!read(reg, old)) return false;
    // A failed transaction may still have reached the chip.
    changes[count++] = {reg, mask, old};
    return write(reg, (old & ~mask) | (value & mask));
  };
  // Preload touch's latch before enabling its output. LCD is released as an
  // input with pull-up/high-Z, not actively driven high. No reset register.
  const bool ready = update(0x05, 0x20, 0x20) &&
                     update(0x0d, 0x10, 0x10) &&
                     update(0x0b, 0x10, 0x10) &&
                     update(0x03, 0x30, 0x20) &&
                     update(0x07, 0x30, 0x10) && verify();
  if (!ready) {
    // Restore only our bits, in reverse order; an unavailable bus prevents a
    // guarantee of rollback, but must not prevent attempts on remaining bits.
    while (count) {
      const auto change = changes[--count];
      uint8_t current = 0;
      if (read(change.reg, current))
        write(change.reg, (current & ~change.mask) | (change.old & change.mask));
    }
  }
  return ready;
}

template<class Read, class Write>
bool tab5_enable_display_touch(Read read, Write write) {
  return tab5_enable_display_touch(read, write, [] { return true; });
}

// --- PWR probe: is any input on this board readable by software at all? -----
//
// The PWR button is invisible to this SDK. Espressif's Tab5 BSP declares no
// buttons (BSP_CAPS_BUTTONS 0) and no battery (BSP_CAPS_BAT 0), this unit's
// console `diag` I2C scan finds no PMIC (0x10 ES8388, 0x14 GT911, 0x32 RX8130CE,
// 0x40/0x41 ES7210, 0x43/0x44 PI4IOE5V6408, 0x68 BMI270), and third-party Tab5
// firmware exposes only the BOOT key (GPIO0). So what could still carry a hidden
// input is:
//   - the PI4IOE5V6408 input status register (0x0F) of both expanders, whose
//     pins reference firmware leaves as inputs: 0x43 P0, and 0x44 P1, P2, P6;
//   - the interrupt status register (0x13) of the same two chips;
//   - a few P4 pads that nothing on this board names.
// `pwrprobe` reads exactly those for a bounded window, printing one line per
// tick and a summary of the bits that changed.
//
// Read-only by construction: the IO contract below has no register write, no
// gpio_set_level and no direction change, so no path here can touch the charger
// or enable pins (0x44 P3/P5/P7, 0x43 P2), the amplifier (0x43 P1), a pin that
// is currently an output, or a pad it did not configure itself. The expanders
// are only ever read, so they end the window exactly as they were found, and a
// pad only ever gains and loses a pull-up.

inline constexpr uint8_t kTab5PwrProbeExpanderA = 0x43;  // BSP_IO_EXPANDER_ADDRESS (low)
inline constexpr uint8_t kTab5PwrProbeExpanderB = 0x44;  // BSP_IO_EXPANDER_ADDRESS_1 (high)
inline constexpr uint8_t kTab5PwrProbeInputReg = 0x0F;   // input status, one bit per pin
inline constexpr uint8_t kTab5PwrProbeIrqReg = 0x13;     // interrupt status, one bit per pin

inline constexpr uint32_t kTab5PwrProbeDefaultSeconds = 10;
inline constexpr uint32_t kTab5PwrProbeMaxSeconds = 30;
inline constexpr uint32_t kTab5PwrProbeTickMs = 200;
// One I2C transaction, the same limit the drivers use: 50 ms covers the wait for
// the bus and for the transfer at the 100 Hz tick.
inline constexpr uint32_t kTab5PwrProbeIoMs = 50;
// Room for the longest line: two registers, four pads and their interrupt status.
inline constexpr size_t kTab5PwrProbeLineBytes = 192;

inline constexpr const char* kTab5PwrProbeUsage =
    "@error usage: pwrprobe [seconds], 1 to 30 (default 10)";
// Appended to the console's `help` on this board only: the probe answers in the
// port (port_console.cpp), not in the app whose help lists the other commands.
inline constexpr const char* kTab5PwrProbeCommandHelp =
    "  pwrprobe [seconds]   Tab5 only: read the 0x0f/0x13 registers of both expanders and a few "
    "unclaimed pads for 1..30 s (default 10); never writes";

// Candidate P4 pads, chosen only because nothing names them: not the BSP pinout
// (0 BOOT, 8-15 C6 SDIO, 19/20 USB host, 22 backlight, 23 touch INT, 26-30 I2S,
// 31/32 I2C, 39-44 microSD), not this board's profile, and not the P4's own USB
// pads (24/25, where the console runs). Nothing here is claimed by construction
// either: a pad that is driven, pulled, open-drain, input-less or mapped to an
// IO_MUX function other than the default or GPIO is skipped when the window
// starts, so this list can only lose candidates, never take one over.
inline constexpr int kTab5PwrProbePins[] = {16, 17, 18, 21};
inline constexpr size_t kTab5PwrProbePinCount =
    sizeof(kTab5PwrProbePins) / sizeof(kTab5PwrProbePins[0]);
static_assert(kTab5PwrProbePinCount <= 8, "one mask byte tracks the candidate pads");

struct Tab5PwrProbeRequest {
  bool ok = false;
  uint32_t seconds = 0;
};

// "pwrprobe" (the default window) or "pwrprobe <seconds>". A window outside
// 1..30 s, or anything that is not a bare number, is refused before a single
// register is read.
inline Tab5PwrProbeRequest tab5_pwr_probe_parse(std::string_view args) {
  Tab5PwrProbeRequest request;
  const auto first = args.find_first_not_of(" \t");
  if (first == std::string_view::npos) {
    request.ok = true;
    request.seconds = kTab5PwrProbeDefaultSeconds;
    return request;
  }
  std::string_view text = args.substr(first);
  text = text.substr(0, text.find_last_not_of(" \t") + 1);
  uint32_t value = 0;
  for (const char c : text) {
    if (c < '0' || c > '9') return request;
    value = value * 10 + static_cast<uint32_t>(c - '0');
    if (value > kTab5PwrProbeMaxSeconds) return request;  // also bounds the loop's reading
  }
  if (value < 1) return request;
  request.ok = true;
  request.seconds = value;
  return request;
}

// What one pad looked like before the probe touched it: the pull state to put
// back, and the evidence that nobody had claimed it.
struct Tab5PwrProbePinConfig {
  bool output = false;    // the GPIO block drives this pad (GPIO_ENABLE)
  bool pull_up = false;   // internal pull-up enabled
  bool pull_down = false;
  bool input = false;     // pad input enable (FUN_IE)
  bool od = false;        // open drain
  bool oe_inv = false;    // inverted output enable
  uint8_t fun_sel = 0;    // IO_MUX function selection
};

// The two IO_MUX selections a pad nobody owns can be in: the chip's reset default
// (0) and the GPIO function the driver selects by name (PIN_FUNC_GPIO, 1 on the
// ESP32-P4). Any other value belongs to another peripheral.
inline constexpr uint8_t kTab5PwrProbeFuncDefault = 0;
inline constexpr uint8_t kTab5PwrProbeFuncGpio = 1;

// A pad belongs to the probe only while nothing else holds it: no drive, no
// pull, no open drain, an input buffer that is already on, and an IO_MUX
// function that is still the default or the GPIO one. Anything else is skipped
// and reported as skipped rather than probed.
//
// Probed pads are only ever given a pull-up and read, never a direction, a level
// or an IO_MUX function, so restoring the pull state restores them exactly. The
// one thing that cannot be restored is a pad a peripheral drives while the
// window runs: it keeps a weak pull-up for at most the window, which changes
// nothing for a driven line and nothing at all afterwards.
inline bool tab5_pwr_probe_pin_free(const Tab5PwrProbePinConfig& pin) {
  return !pin.output && !pin.pull_up && !pin.pull_down && !pin.od && !pin.oe_inv && pin.input &&
         (pin.fun_sel == kTab5PwrProbeFuncDefault || pin.fun_sel == kTab5PwrProbeFuncGpio);
}

// One tick's reads. A source that did not answer keeps its flag false, and the
// line then shows "-" instead of a level nobody measured.
struct Tab5PwrProbeSample {
  // Index 0 is expander 0x43, index 1 is 0x44 (tab5_pwr_probe_index).
  bool in_ok[2] = {false, false};
  uint8_t in[2] = {0, 0};
  bool irq_ok[2] = {false, false};
  uint8_t irq[2] = {0, 0};
  uint8_t gpio_high = 0;  // bit per candidate pad, in kTab5PwrProbePins order
  uint8_t gpio_ok = 0;    // which candidate pads contributed a level
};

inline constexpr size_t tab5_pwr_probe_index(uint8_t expander) {
  return expander == kTab5PwrProbeExpanderB ? 1u : 0u;
}

// What changed during the window: the bits that differed at some point, so both
// edges of a press count, and which sources answered at all.
class Tab5PwrProbeTracker {
 public:
  void observe(const Tab5PwrProbeSample& sample) {
    bool irq_changed_now = false;
    for (size_t i = 0; i < 2; ++i) {
      if (sample.in_ok[i]) {
        if (seen_in_[i]) changed_in_[i] = static_cast<uint8_t>(changed_in_[i] | (previous_in_[i] ^ sample.in[i]));
        previous_in_[i] = sample.in[i];
        seen_in_[i] = true;
      }
      if (sample.irq_ok[i]) {
        if (seen_irq_[i]) {
          const uint8_t diff = static_cast<uint8_t>(previous_irq_[i] ^ sample.irq[i]);
          changed_irq_[i] = static_cast<uint8_t>(changed_irq_[i] | diff);
          irq_changed_now = irq_changed_now || diff != 0;
        }
        previous_irq_[i] = sample.irq[i];
        seen_irq_[i] = true;
      }
    }
    if (sample.gpio_ok) {
      if (seen_gpio_) changed_gpio_ = static_cast<uint8_t>(changed_gpio_ | ((previous_gpio_ ^ sample.gpio_high) & sample.gpio_ok));
      previous_gpio_ = sample.gpio_high;
      seen_gpio_ = true;
    }
    ++ticks_;
    // The first tick always carries 0x13, so the operator sees a baseline.
    irq_changed_ = ticks_ == 1 || irq_changed_now;
  }

  uint32_t ticks() const { return ticks_; }
  // True when this tick's 0x13 value differs from the previous one.
  bool irq_changed() const { return irq_changed_; }
  uint8_t changed_in(uint8_t expander) const { return changed_in_[tab5_pwr_probe_index(expander)]; }
  uint8_t changed_irq(uint8_t expander) const { return changed_irq_[tab5_pwr_probe_index(expander)]; }
  bool saw_in(uint8_t expander) const { return seen_in_[tab5_pwr_probe_index(expander)]; }
  bool saw_irq(uint8_t expander) const { return seen_irq_[tab5_pwr_probe_index(expander)]; }
  uint8_t changed_gpio() const { return changed_gpio_; }
  bool saw_gpio() const { return seen_gpio_; }
  bool any_change() const {
    return changed_in_[0] || changed_in_[1] || changed_irq_[0] || changed_irq_[1] || changed_gpio_;
  }

 private:
  uint8_t previous_in_[2] = {0, 0};
  uint8_t previous_irq_[2] = {0, 0};
  bool seen_in_[2] = {false, false};
  bool seen_irq_[2] = {false, false};
  uint8_t changed_in_[2] = {0, 0};
  uint8_t changed_irq_[2] = {0, 0};
  bool seen_gpio_ = false;
  uint8_t previous_gpio_ = 0;
  uint8_t changed_gpio_ = 0;
  uint32_t ticks_ = 0;
  bool irq_changed_ = false;
};

// Bounded string building for the three lines below: overflow drops the line
// instead of printing a truncated one.
class Tab5PwrProbeText {
 public:
  Tab5PwrProbeText(char* out, size_t capacity) : out_(out), capacity_(capacity) {}
  void add(const char* format, ...) {
    if (full_ || !out_ || capacity_ == 0 || used_ >= capacity_) {
      full_ = true;
      return;
    }
    va_list args;
    va_start(args, format);
    const int written = std::vsnprintf(out_ + used_, capacity_ - used_, format, args);
    va_end(args);
    if (written < 0 || static_cast<size_t>(written) >= capacity_ - used_) {
      full_ = true;
      return;
    }
    used_ += static_cast<size_t>(written);
  }
  // The line's length, or 0 when it did not fit.
  size_t done() const { return full_ ? 0 : used_; }

 private:
  char* out_;
  size_t capacity_;
  size_t used_ = 0;
  bool full_ = false;
};

// The window's opening line: how long it runs, which registers it reads and
// which pads it will leave alone.
inline size_t tab5_pwr_probe_header(char* out, size_t capacity, uint32_t seconds, uint8_t probed,
                                    uint8_t skipped) {
  Tab5PwrProbeText text(out, capacity);
  text.add("pwrprobe: %us; expanders 0x%02x 0x%02x regs 0x%02x/0x%02x; pads", unsigned(seconds),
           unsigned(kTab5PwrProbeExpanderA), unsigned(kTab5PwrProbeExpanderB),
           unsigned(kTab5PwrProbeInputReg), unsigned(kTab5PwrProbeIrqReg));
  unsigned listed = 0;
  for (size_t i = 0; i < kTab5PwrProbePinCount; ++i) {
    if (!(probed & (1u << i))) continue;
    text.add(" %d", kTab5PwrProbePins[i]);
    ++listed;
  }
  if (!listed) text.add(" none");
  listed = 0;
  for (size_t i = 0; i < kTab5PwrProbePinCount; ++i) {
    if (!(skipped & (1u << i))) continue;
    text.add(listed ? " %d" : " (skipped %d", kTab5PwrProbePins[i]);
    ++listed;
  }
  if (listed) text.add(")");
  return text.done();
}

// One compact line per tick, e.g.
//   pwrprobe 0.6s 0x43 in=0x00 0x44 in=0x08 pads 16=1 17=0 18=-
// with the interrupt status appended on the tick where it changed:
//   pwrprobe 1.2s ... irq 0x43=0x00 0x44=0x08
inline size_t tab5_pwr_probe_line(char* out, size_t capacity, const Tab5PwrProbeSample& sample,
                                  const Tab5PwrProbeTracker& tracker, uint32_t elapsed_ms) {
  Tab5PwrProbeText text(out, capacity);
  text.add("pwrprobe %u.%us", unsigned(elapsed_ms / 1000), unsigned((elapsed_ms % 1000) / 100));
  for (size_t i = 0; i < 2; ++i) {
    const uint8_t expander = i == 0 ? kTab5PwrProbeExpanderA : kTab5PwrProbeExpanderB;
    if (sample.in_ok[i]) text.add(" 0x%02x in=0x%02x", unsigned(expander), unsigned(sample.in[i]));
    else text.add(" 0x%02x in=-", unsigned(expander));
  }
  text.add(" pads");
  unsigned listed = 0;
  for (size_t i = 0; i < kTab5PwrProbePinCount; ++i) {
    if (!(sample.gpio_ok & (1u << i))) continue;
    text.add(" %d=%d", kTab5PwrProbePins[i], (sample.gpio_high >> i) & 1u);
    ++listed;
  }
  if (!listed) text.add(" none");
  if (tracker.irq_changed()) {
    text.add(" irq");
    for (size_t i = 0; i < 2; ++i) {
      const uint8_t expander = i == 0 ? kTab5PwrProbeExpanderA : kTab5PwrProbeExpanderB;
      if (sample.irq_ok[i]) text.add(" 0x%02x=0x%02x", unsigned(expander), unsigned(sample.irq[i]));
      else text.add(" 0x%02x=-", unsigned(expander));
    }
  }
  return text.done();
}

// The window's closing line: which bits changed, so a press anywhere in the
// window shows up even when it ended before the next tick.
inline size_t tab5_pwr_probe_summary(char* out, size_t capacity, const Tab5PwrProbeTracker& tracker) {
  Tab5PwrProbeText text(out, capacity);
  text.add("pwrprobe summary: %u ticks; changed", unsigned(tracker.ticks()));
  for (size_t i = 0; i < 2; ++i) {
    const uint8_t expander = i == 0 ? kTab5PwrProbeExpanderA : kTab5PwrProbeExpanderB;
    if (tracker.saw_in(expander)) text.add(" 0x%02x in=0x%02x", unsigned(expander), unsigned(tracker.changed_in(expander)));
    else text.add(" 0x%02x in=-", unsigned(expander));
  }
  for (size_t i = 0; i < 2; ++i) {
    const uint8_t expander = i == 0 ? kTab5PwrProbeExpanderA : kTab5PwrProbeExpanderB;
    if (tracker.saw_irq(expander)) text.add(" 0x%02x irq=0x%02x", unsigned(expander), unsigned(tracker.changed_irq(expander)));
    else text.add(" 0x%02x irq=-", unsigned(expander));
  }
  text.add(" pads=");
  unsigned listed = 0;
  for (size_t i = 0; i < kTab5PwrProbePinCount; ++i) {
    if (!(tracker.changed_gpio() & (1u << i))) continue;
    text.add(listed ? ",%d" : "%d", kTab5PwrProbePins[i]);
    ++listed;
  }
  if (!listed) text.add("-");
  if (!tracker.any_change()) text.add(" (nothing changed)");
  return text.done();
}

// The IO an ESP adapter must provide. There is deliberately no register write
// and no way to drive a pad:
//   bool read(uint8_t expander, uint8_t reg, uint8_t& value, uint32_t timeout_ms)
//   uint64_t now_ms();
//   void delay(uint32_t ms);
//   void print(const char* line);
//   bool pin_state(int gpio, Tab5PwrProbePinConfig& state);
//   bool pin_pull_up(int gpio);   // internal pull-up on, and nothing else
//   void pin_restore(int gpio, const Tab5PwrProbePinConfig& state);  // pull only
//   int pin_level(int gpio);  // 0, 1, or negative when the level cannot be read
//
// One window per instance: a second call refuses, so a repeat needs a new
// command. Nothing here is reentrant; the caller owns the bus and the pads.
template <class IO>
class Tab5PwrProbe {
 public:
  explicit Tab5PwrProbe(IO& io) : io_(io) {}
  Tab5PwrProbe(const Tab5PwrProbe&) = delete;
  Tab5PwrProbe& operator=(const Tab5PwrProbe&) = delete;

  // Runs the whole window and leaves every register and pad as it found them.
  // False only when the window was refused, in which case the usage line is the
  // only thing printed and no IO happened.
  bool run(std::string_view args) {
    if (used_) return false;
    used_ = true;
    const Tab5PwrProbeRequest request = tab5_pwr_probe_parse(args);
    if (!request.ok) {
      io_.print(kTab5PwrProbeUsage);
      return false;
    }
    prepare_pads();
    char line[kTab5PwrProbeLineBytes];
    if (const size_t n = tab5_pwr_probe_header(line, sizeof(line), request.seconds, probed_, skipped_))
      io_.print(line);
    const uint32_t window_ms = request.seconds * 1000;
    // A clock that stops advancing must not turn the window into a loop: the
    // tick budget ends it too, and each tick then charges its own delay.
    const uint32_t max_ticks = request.seconds * (1000 / kTab5PwrProbeTickMs) + 4;
    const uint64_t start = io_.now_ms();
    uint32_t elapsed = 0;
    for (uint32_t i = 0; i < max_ticks; ++i) {
      read_sample();
      tracker_.observe(sample_);
      if (const size_t n = tab5_pwr_probe_line(line, sizeof(line), sample_, tracker_, elapsed)) io_.print(line);
      if (elapsed + kTab5PwrProbeTickMs >= window_ms) break;
      io_.delay(kTab5PwrProbeTickMs);
      const uint64_t ran = io_.now_ms() - start;
      const uint32_t advanced = ran > 0xffffffffull ? 0xffffffffu : static_cast<uint32_t>(ran);
      elapsed = advanced > elapsed ? advanced : elapsed + kTab5PwrProbeTickMs;
    }
    restore_pads();
    if (const size_t n = tab5_pwr_probe_summary(line, sizeof(line), tracker_)) io_.print(line);
    return true;
  }

 private:
  void prepare_pads() {
    for (size_t i = 0; i < kTab5PwrProbePinCount; ++i) {
      Tab5PwrProbePinConfig state;
      if (!io_.pin_state(kTab5PwrProbePins[i], state) || !tab5_pwr_probe_pin_free(state) ||
          !io_.pin_pull_up(kTab5PwrProbePins[i])) {
        skipped_ = static_cast<uint8_t>(skipped_ | (1u << i));
        continue;
      }
      states_[i] = state;
      probed_ = static_cast<uint8_t>(probed_ | (1u << i));
    }
  }
  void restore_pads() {
    for (size_t i = 0; i < kTab5PwrProbePinCount; ++i)
      if (probed_ & (1u << i)) io_.pin_restore(kTab5PwrProbePins[i], states_[i]);
  }
  void read_sample() {
    sample_ = Tab5PwrProbeSample{};
    for (size_t i = 0; i < 2; ++i) {
      const uint8_t expander = i == 0 ? kTab5PwrProbeExpanderA : kTab5PwrProbeExpanderB;
      sample_.in_ok[i] = io_.read(expander, kTab5PwrProbeInputReg, sample_.in[i], kTab5PwrProbeIoMs);
      sample_.irq_ok[i] = io_.read(expander, kTab5PwrProbeIrqReg, sample_.irq[i], kTab5PwrProbeIoMs);
    }
    for (size_t i = 0; i < kTab5PwrProbePinCount; ++i) {
      if (!(probed_ & (1u << i))) continue;
      const int level = io_.pin_level(kTab5PwrProbePins[i]);
      if (level < 0) continue;
      sample_.gpio_ok = static_cast<uint8_t>(sample_.gpio_ok | (1u << i));
      if (level) sample_.gpio_high = static_cast<uint8_t>(sample_.gpio_high | (1u << i));
    }
  }

  IO& io_;
  bool used_ = false;
  uint8_t probed_ = 0;
  uint8_t skipped_ = 0;
  Tab5PwrProbePinConfig states_[kTab5PwrProbePinCount] = {};
  Tab5PwrProbeSample sample_;
  Tab5PwrProbeTracker tracker_;
};

// --- Battery: is the INA226 on this board's bus? -----------------------------
//
// Espressif's Tab5 README names an IP2326 charge/discharge controller, an INA226
// monitor and a removable NP-F550 cell for this board, but the BSP declares
// BSP_CAPS_BAT 0, this repo has no INA226 driver, and this unit's console `diag`
// I2C scan finds 0x40 and 0x41 -- where the ES7210 microphones live, and where
// 0x40 is also the INA226's default address. So `batt` asks the chip itself
// instead of trusting the README: the manufacturer id register (0xfe, 0x5449 for
// TI) and the die id register (0xff, 0x2260 for an INA226) decide which
// candidate, if any, is one before a single register is written.
//
// What it then does is one shot and bounded: the calibration and the config
// register that starts ONE triggered shunt+bus conversion, one fixed wait, then
// the four result registers once each. No polling, no repeated conversion, no
// loop. The only writes in the whole command go to the chip that identified
// itself, at 0x40 or 0x41; the charger enable pins (expander 0x44 P3/P5/P7), the
// amplifier (0x43 P1) and every other chip on the bus are never written.
//
// Two documented defaults, both assumptions this command prints:
//   - the shunt is 20 mOhm. `batt <milliohm>` (1..1000) overrides it; voltage
//     readings do not depend on it, current and power scale inversely with it.
//   - a positive current register means charging, a negative one discharging,
//     which follows how IN+/IN- sit across the shunt. A unit that reads a charge
//     as negative has the two swapped, and the sign, not the magnitude, is wrong.
// Neither is a fuel gauge: a percentage would be estimated from the bus voltage.

inline constexpr uint8_t kTab5BattAddressA = 0x40;  // INA226's default address, also an ES7210
inline constexpr uint8_t kTab5BattAddressB = 0x41;  // the second ES7210: asked the same question
inline constexpr uint8_t kTab5InaRegConfig = 0x00;
inline constexpr uint8_t kTab5InaRegShunt = 0x01;        // signed, 2.5 uV per LSB
inline constexpr uint8_t kTab5InaRegBus = 0x02;          // unsigned, 1.25 mV per LSB
inline constexpr uint8_t kTab5InaRegPower = 0x03;        // unsigned, 25 current LSBs per LSB
inline constexpr uint8_t kTab5InaRegCurrent = 0x04;      // signed, one current LSB per LSB
inline constexpr uint8_t kTab5InaRegCalibration = 0x05;
inline constexpr uint8_t kTab5InaRegManufacturer = 0xFE;
inline constexpr uint8_t kTab5InaRegDie = 0xFF;
inline constexpr uint16_t kTab5InaManufacturerTi = 0x5449;  // 'T','I'
inline constexpr uint16_t kTab5InaDieId = 0x2260;           // the INA226 itself
// RST=0, AVG=000 (one sample), VBUSCT=VSHCT=000 (140 us each), MODE=011: shunt
// and bus, triggered. Writing it starts exactly one conversion pair and the chip
// returns to power-down afterwards, so nothing keeps converting on its own.
inline constexpr uint16_t kTab5InaConfigOneShot = 0x0003;
// 0.00512 / (current LSB * shunt). With the LSB below, that divides out to this
// constant for every shunt: 0.00512 / (2.5 uV / R * R) = 2048.
inline constexpr uint16_t kTab5InaCalibration = 2048;
// The INA226's own LSBs, in the integer units the formatting uses.
inline constexpr uint32_t kTab5InaBusLsbUv = 1250;   // 1.25 mV
inline constexpr uint32_t kTab5InaShuntLsbNv = 2500; // 2.5 uV
inline constexpr uint32_t kTab5BattDefaultShuntMilliohm = 20;
inline constexpr uint32_t kTab5BattMinShuntMilliohm = 1;
inline constexpr uint32_t kTab5BattMaxShuntMilliohm = 1000;

// The bus register (0x02) is 1.25 mV per LSB, unsigned. This is the uint16_t
// field hal.power carries, so the value is rounded and saturates instead of
// wrapping: a register at full scale is 81.9 V, which does not fit. Only a broken
// chip reads that high; the console `batt` line keeps the full 81919 mV.
inline uint16_t tab5_ina_bus_mv(uint16_t raw) {
  const uint32_t mv = (static_cast<uint32_t>(raw) * kTab5InaBusLsbUv + 500) / 1000;
  return mv > 0xffffu ? static_cast<uint16_t>(0xffffu) : static_cast<uint16_t>(mv);
}

// One I2C transaction, the same limit the drivers and the PWR probe use.
inline constexpr uint32_t kTab5BattIoMs = 50;
// One triggered conversion at 140 us per channel, with AVG=1: this wait is two
// orders of magnitude longer than the conversion and is not polled.
inline constexpr uint32_t kTab5InaConversionWaitMs = 10;
// Room for the longest line: the reading, with every field at full width.
inline constexpr size_t kTab5BattLineBytes = 192;

inline constexpr const char* kTab5BattUsage = "@error usage: batt [shunt_milliohm], 1 to 1000 (default 20)";
// Appended to the console's `help` on this board only, like the PWR probe's line.
inline constexpr const char* kTab5BattCommandHelp =
    "  batt [milliohm]      Tab5 only: ask 0x40/0x41 for the INA226 ids, then one triggered reading (bus mV, "
    "shunt uV, current mA, power mW); writes only that chip's own config";

// What one candidate address answered.
enum class Tab5InaVerdict : uint8_t { NoAnswer, NotIna226, Ina226 };

// The manufacturer id alone is not enough: 0x5449 says TI, not which TI part, so
// the die id has to name the INA226 too. A chip that answered one and not the
// other is not trusted with a write.
inline Tab5InaVerdict tab5_ina_verdict(bool answered, uint16_t manufacturer, uint16_t die) {
  if (!answered) return Tab5InaVerdict::NoAnswer;
  return manufacturer == kTab5InaManufacturerTi && die == kTab5InaDieId ? Tab5InaVerdict::Ina226
                                                                        : Tab5InaVerdict::NotIna226;
}

// The current register's LSB in nanoamp for a shunt in milliohm: 2.5 uV / R, so
// 2.5 mA per ohm -- 125 uA at the default 20 mOhm, and the "current lsb" the
// setup line prints.
inline int32_t tab5_ina_current_lsb_na(uint32_t shunt_milliohm) {
  return shunt_milliohm ? static_cast<int32_t>(2500000u / shunt_milliohm) : 0;
}

struct Tab5BattRequest {
  bool ok = false;
  uint32_t shunt_milliohm = 0;
};

// "batt" (the default shunt) or "batt <milliohm>", 1..1000. Anything else is
// refused before a single register is read.
inline Tab5BattRequest tab5_batt_parse(std::string_view args) {
  Tab5BattRequest request;
  const auto first = args.find_first_not_of(" \t");
  if (first == std::string_view::npos) {
    request.ok = true;
    request.shunt_milliohm = kTab5BattDefaultShuntMilliohm;
    return request;
  }
  std::string_view text = args.substr(first);
  text = text.substr(0, text.find_last_not_of(" \t") + 1);
  uint32_t value = 0;
  for (const char c : text) {
    if (c < '0' || c > '9') return request;
    value = value * 10 + static_cast<uint32_t>(c - '0');
    if (value > kTab5BattMaxShuntMilliohm) return request;
  }
  if (value < kTab5BattMinShuntMilliohm) return request;
  request.ok = true;
  request.shunt_milliohm = value;
  return request;
}

struct Tab5BattCandidate {
  uint8_t address = 0;
  bool answered = false;      // the manufacturer id read came back
  uint16_t manufacturer = 0;
  bool die_answered = false;  // the die id read came back
  uint16_t die = 0;
  Tab5InaVerdict verdict = Tab5InaVerdict::NoAnswer;
};

// The four result registers, as the chip returned them.
struct Tab5InaReading {
  uint16_t bus_raw = 0;      // 0x02
  int16_t shunt_raw = 0;     // 0x01
  int16_t current_raw = 0;   // 0x04
  uint16_t power_raw = 0;    // 0x03
};

// "+ charges the cell, - discharges it": the sign of the current register.
inline const char* tab5_ina_sign_text(int16_t current_raw) {
  if (current_raw > 0) return "charge";
  if (current_raw < 0) return "discharge";
  return "idle";
}

// A signed value in tenths, as "-12.5". Bounded like every other line here: an
// overflow drops the text instead of cutting it.
inline size_t tab5_batt_fixed_tenths(char* out, size_t capacity, int64_t tenths) {
  Tab5PwrProbeText text(out, capacity);
  const bool negative = tenths < 0;
  const uint64_t magnitude = static_cast<uint64_t>(negative ? -tenths : tenths);
  text.add("%s%llu.%llu", negative ? "-" : "", static_cast<unsigned long long>(magnitude / 10),
           static_cast<unsigned long long>(magnitude % 10));
  return text.done();
}

// The line that opens every run: what is asked where, and the shunt assumed.
inline size_t tab5_batt_header(char* out, size_t capacity, uint32_t shunt_milliohm) {
  Tab5PwrProbeText text(out, capacity);
  text.add("batt: probing 0x%02x 0x%02x for INA226 (mfr 0xfe=0x%04x, die 0xff=0x%04x); shunt %u mOhm",
           unsigned(kTab5BattAddressA), unsigned(kTab5BattAddressB), unsigned(kTab5InaManufacturerTi),
           unsigned(kTab5InaDieId), unsigned(shunt_milliohm));
  return text.done();
}

// One line per candidate, including the one that said nothing: a silent address
// is a result too, and "-" never stands in for a value nobody read.
inline size_t tab5_batt_candidate_line(char* out, size_t capacity, const Tab5BattCandidate& candidate) {
  Tab5PwrProbeText text(out, capacity);
  if (candidate.verdict == Tab5InaVerdict::NoAnswer) {
    text.add("batt: 0x%02x no answer", unsigned(candidate.address));
    return text.done();
  }
  text.add("batt: 0x%02x mfr=0x%04x die=", unsigned(candidate.address), unsigned(candidate.manufacturer));
  if (candidate.die_answered) text.add("0x%04x", unsigned(candidate.die));
  else text.add("-");
  text.add(candidate.verdict == Tab5InaVerdict::Ina226 ? ": INA226" : ": not an INA226");
  return text.done();
}

inline size_t tab5_batt_none_line(char* out, size_t capacity) {
  Tab5PwrProbeText text(out, capacity);
  text.add("batt: no INA226 here: neither 0x%02x nor 0x%02x answered as one", unsigned(kTab5BattAddressA),
           unsigned(kTab5BattAddressB));
  return text.done();
}

// What was written, with the two documented defaults spelled out.
inline size_t tab5_batt_setup_line(char* out, size_t capacity, uint8_t address, uint32_t shunt_milliohm) {
  Tab5PwrProbeText text(out, capacity);
  const int32_t lsb_na = tab5_ina_current_lsb_na(shunt_milliohm);
  text.add("batt: 0x%02x config 0x%04x (shunt+bus, one triggered conversion), cal %u (current lsb %u.%03u mA)",
           unsigned(address), unsigned(kTab5InaConfigOneShot), unsigned(kTab5InaCalibration),
           unsigned(lsb_na / 1000000), unsigned((lsb_na % 1000000) / 1000));
  return text.done();
}

// The reading: bus voltage in mV, shunt voltage in uV, current in mA with its
// direction, and power in mW, each converted from the chip's own LSB.
inline size_t tab5_batt_reading(char* out, size_t capacity, uint8_t address, const Tab5InaReading& reading,
                                uint32_t shunt_milliohm) {
  // The console prints whole millivolts without a ceiling (an int64 holds the
  // full-scale 81919 mV too); the uint16_t contract field the indicator uses has
  // its own saturating conversion, tab5_ina_bus_mv.
  const int64_t bus_uv = static_cast<int64_t>(reading.bus_raw) * kTab5InaBusLsbUv;
  const int64_t bus_mv = (bus_uv + 500) / 1000;  // 1.25 mV per LSB, shown whole
  const int64_t shunt_tenths_uv = static_cast<int64_t>(reading.shunt_raw) * (kTab5InaShuntLsbNv / 100);
  const int32_t lsb_na = tab5_ina_current_lsb_na(shunt_milliohm);
  const int64_t current_na = static_cast<int64_t>(reading.current_raw) * lsb_na;
  // Rounded to a tenth of a mA, the resolution the default shunt reads at.
  const int64_t current_tenths_ma = (current_na + (current_na < 0 ? -50000 : 50000)) / 100000;
  // The power register counts 25 current LSBs: 3.125 mW per LSB at 20 mOhm.
  const int64_t power_uw = static_cast<int64_t>(reading.power_raw) * 25 * lsb_na / 1000;
  const int64_t power_mw = (power_uw + 500) / 1000;
  char shunt[24];
  char current[24];
  if (tab5_batt_fixed_tenths(shunt, sizeof(shunt), shunt_tenths_uv) == 0) return 0;
  if (tab5_batt_fixed_tenths(current, sizeof(current), current_tenths_ma) == 0) return 0;
  Tab5PwrProbeText text(out, capacity);
  text.add("batt: 0x%02x bus %lld mV | shunt %s uV | current %s mA (%s) | power %lld mW", unsigned(address),
           static_cast<long long>(bus_mv), shunt, current, tab5_ina_sign_text(reading.current_raw),
           static_cast<long long>(power_mw));
  return text.done();
}

// The one line that keeps the reading honest: the sign is a wiring assumption,
// and a percentage from these numbers would be an estimate, not a gauge.
inline size_t tab5_batt_sign_note(char* out, size_t capacity) {
  Tab5PwrProbeText text(out, capacity);
  text.add("batt: + charges the cell, - discharges it; a percentage here would be an estimate from bus voltage, "
           "not a fuel gauge");
  return text.done();
}

inline size_t tab5_batt_failure_line(char* out, size_t capacity, uint8_t address, bool write, uint8_t reg) {
  Tab5PwrProbeText text(out, capacity);
  text.add("batt: 0x%02x %s of reg 0x%02x failed: no reading", unsigned(address), write ? "write" : "read",
           unsigned(reg));
  return text.done();
}

// The IO an ESP adapter must provide:
//   bool read16(uint8_t address, uint8_t reg, uint16_t& value, uint32_t timeout_ms)
//   bool write16(uint8_t address, uint8_t reg, uint16_t value, uint32_t timeout_ms)
//   void delay(uint32_t ms);
//   void print(const char* line);
// False from either call means the transfer failed (a NAK or a timeout); the
// command then reports no reading rather than inventing one.
template <class IO>
class Tab5BatteryProbe {
 public:
  explicit Tab5BatteryProbe(IO& io) : io_(io) {}
  Tab5BatteryProbe(const Tab5BatteryProbe&) = delete;
  Tab5BatteryProbe& operator=(const Tab5BatteryProbe&) = delete;

  // One run per instance: a repeat needs a new command, so nothing here can
  // convert twice or keep the bus busy.
  bool run(std::string_view args) {
    if (used_) return false;
    used_ = true;
    const Tab5BattRequest request = tab5_batt_parse(args);
    if (!request.ok) {
      io_.print(kTab5BattUsage);
      return false;
    }
    char line[kTab5BattLineBytes];
    if (const size_t n = tab5_batt_header(line, sizeof(line), request.shunt_milliohm)) io_.print(line);
    // Both candidates are always asked, so the report says what each one is.
    const uint8_t addresses[2] = {kTab5BattAddressA, kTab5BattAddressB};
    Tab5BattCandidate found;
    bool have = false;
    for (const uint8_t address : addresses) {
      const Tab5BattCandidate candidate = identify(address);
      if (const size_t n = tab5_batt_candidate_line(line, sizeof(line), candidate)) io_.print(line);
      if (!have && candidate.verdict == Tab5InaVerdict::Ina226) {
        found = candidate;
        have = true;
      }
    }
    if (!have) {
      if (const size_t n = tab5_batt_none_line(line, sizeof(line))) io_.print(line);
      return true;
    }
    const uint8_t address = found.address;
    const auto failed = [&](bool write, uint8_t reg) {
      if (const size_t n = tab5_batt_failure_line(line, sizeof(line), address, write, reg)) io_.print(line);
      return true;
    };
    // Calibration first: it only scales the current and power registers, so it
    // can precede the conversion. Then the config write, which starts it.
    if (!io_.write16(address, kTab5InaRegCalibration, kTab5InaCalibration, kTab5BattIoMs))
      return failed(true, kTab5InaRegCalibration);
    if (!io_.write16(address, kTab5InaRegConfig, kTab5InaConfigOneShot, kTab5BattIoMs))
      return failed(true, kTab5InaRegConfig);
    if (const size_t n = tab5_batt_setup_line(line, sizeof(line), address, request.shunt_milliohm)) io_.print(line);
    io_.delay(kTab5InaConversionWaitMs);  // one fixed wait, never a poll of the ready flag
    Tab5InaReading reading;
    uint16_t raw = 0;
    if (!io_.read16(address, kTab5InaRegBus, raw, kTab5BattIoMs)) return failed(false, kTab5InaRegBus);
    reading.bus_raw = raw;
    if (!io_.read16(address, kTab5InaRegShunt, raw, kTab5BattIoMs)) return failed(false, kTab5InaRegShunt);
    reading.shunt_raw = static_cast<int16_t>(raw);
    if (!io_.read16(address, kTab5InaRegCurrent, raw, kTab5BattIoMs)) return failed(false, kTab5InaRegCurrent);
    reading.current_raw = static_cast<int16_t>(raw);
    if (!io_.read16(address, kTab5InaRegPower, raw, kTab5BattIoMs)) return failed(false, kTab5InaRegPower);
    reading.power_raw = raw;
    if (const size_t n = tab5_batt_reading(line, sizeof(line), address, reading, request.shunt_milliohm))
      io_.print(line);
    if (const size_t n = tab5_batt_sign_note(line, sizeof(line))) io_.print(line);
    return true;
  }

 private:
  // The two identity reads, in this order: manufacturer, then die. An address
  // that does not answer the first is not asked again.
  Tab5BattCandidate identify(uint8_t address) {
    Tab5BattCandidate candidate;
    candidate.address = address;
    uint16_t value = 0;
    candidate.answered = io_.read16(address, kTab5InaRegManufacturer, value, kTab5BattIoMs);
    if (!candidate.answered) return candidate;
    candidate.manufacturer = value;
    candidate.die_answered = io_.read16(address, kTab5InaRegDie, value, kTab5BattIoMs);
    if (candidate.die_answered) candidate.die = value;
    candidate.verdict = tab5_ina_verdict(candidate.die_answered, candidate.manufacturer, candidate.die);
    return candidate;
  }

  IO& io_;
  bool used_ = false;
};

// --- Battery indicator: the INA226's bus voltage as hal.power -----------------
//
// Measured on this unit with an NP-F970 pack fitted: the INA226 at 0x41 reads
// 8330 mV with USB connected (a 2S Li-ion pack: 7.2 V nominal, 8.4 V full) and
// 1213 mV with the pack disconnected. That is all this board can measure: the
// BSP declares no PMIC and no battery capability, there is no power key, and the
// shunt current read about 0.1 mA -- far below anything a charge or discharge
// draws -- so a positive or negative current register cannot be trusted to say
// which is happening.
//
// So the port reports what it can: the bus voltage, whether a pack is fitted at
// all, and a percentage derived from that voltage through the 2S discharge curve
// below. The percentage is a voltage estimate, NOT a fuel gauge: it has no
// Coulomb counting, a pack that has just been charging reads high, a pack under
// load reads low, and the two cells' imbalance is not seen. `charging` and
// `external_power` stay unknown rather than guessed, which the contract already
// handles (unknown counts as plugged in for doze decisions).

// A poll is two short I2C transfers and one fixed wait: it must never hold the
// app task long enough to be visible on the screen. Shorter than the console
// command's 50 ms, which has no UI behind it.
inline constexpr uint32_t kTab5PowerIoMs = 20;

// 2S Li-ion resting voltage: 8.4 V (4.2 V/cell) full to 6.4 V (3.2 V/cell) empty,
// doubled from the usual per-cell open-circuit curve. Between points the line is
// straight; above the first point 100 and below the last 0, so the result is
// clamped at both ends.
inline constexpr hg::CurvePoint kTab5Pack2sPoints[] = {
    {8400, 100}, {8200, 90}, {8000, 80}, {7860, 70}, {7740, 60}, {7600, 50},
    {7500, 40},  {7400, 30}, {7300, 20}, {7000, 10}, {6600, 5},  {6400, 0},
};
inline constexpr hg::Curve kTab5Pack2sCurve{kTab5Pack2sPoints,
                                             sizeof(kTab5Pack2sPoints) / sizeof(kTab5Pack2sPoints[0])};

// Pack present or not is decided by voltage, not by the register answering: with
// the cells removed the pack terminal reads 1213 mV on this unit (the charger's
// rail through the pack's own divider), while a protected 2S pack cuts off
// around 5.4-5.5 V. 5.0 V is below any fitted pack and far above an empty
// terminal, so neither a missing pack nor a deeply discharged one is misreported.
inline constexpr uint16_t kTab5PackPresentMv = 5000;

// The percentage a 2S pack holds at this voltage, from the curve above.
inline uint8_t tab5_pack_percent(uint16_t mv) { return hg::percent_on_curve(kTab5Pack2sCurve, mv); }

inline bool tab5_pack_present(uint16_t mv) { return mv >= kTab5PackPresentMv; }

// What one poll found. `percent` is the 2S voltage estimate above. There is
// deliberately no charge and no USB field: the shunt cannot tell the two apart,
// and a port that filled them anyway would be guessing. The ESP adapter in
// main.cpp maps exactly these three onto hg::PowerStatus.
struct Tab5BatteryReading {
  uint16_t mv = 0;
  bool present = false;
  std::optional<uint8_t> percent;
};

// Logged once when the chip identifies itself, and once per failure transition
// rather than on every 5 s poll: the reading is an estimate, and a chip that
// stopped answering must not look like a battery that went flat.
inline constexpr const char* kTab5PowerEstimateLog =
    "Tab5 power: INA226 bus voltage, and the percent from it, is a voltage estimate (2S Li-ion, 8.4 V full, 6.4 V "
    "empty), not a fuel gauge; charging and USB power stay unknown because the shunt cannot tell a charge from a "
    "discharge";
inline constexpr const char* kTab5PowerAbsentLog =
    "Tab5 power: no INA226 answered at 0x40 or 0x41; no battery indicator and no battery settings this boot";
inline constexpr const char* kTab5PowerFailedLog =
    "Tab5 power: the INA226 did not answer; no battery reading this poll, not the last one";
inline constexpr const char* kTab5PowerRecoveredLog = "Tab5 power: the INA226 answered again";

// The IO an ESP adapter must provide:
//   bool read16(uint8_t address, uint8_t reg, uint16_t& value, uint32_t timeout_ms)
//   bool write16(uint8_t address, uint8_t reg, uint16_t value, uint32_t timeout_ms)
//   void delay(uint32_t ms);
//   void print(const char* line);
// False from either transfer means the chip did not answer (a NAK or a timeout);
// the poll then reports no battery rather than a remembered one. There is no
// expander register and no GPIO in this contract, so the charger enable pins,
// the amplifier and the audio codec are unreachable from here.
template <class IO>
class Tab5PowerReader {
 public:
  explicit Tab5PowerReader(IO& io) : io_(io) {}
  Tab5PowerReader(const Tab5PowerReader&) = delete;
  Tab5PowerReader& operator=(const Tab5PowerReader&) = delete;

  // Identifies the monitor once, before anything is written: the manufacturer
  // (0xfe) and die (0xff) ids at 0x40 and then 0x41, the addresses this unit's
  // bus scan puts the microphone codec and the INA226 at. False means the
  // caller registers no power port at all, so a board without a monitor offers
  // no battery indicator and no battery settings.
  bool begin() {
    if (ready_) return true;
    const uint8_t addresses[2] = {kTab5BattAddressA, kTab5BattAddressB};
    for (const uint8_t address : addresses) {
      uint16_t manufacturer = 0;
      uint16_t die = 0;
      const bool answered = io_.read16(address, kTab5InaRegManufacturer, manufacturer, kTab5PowerIoMs) &&
                            io_.read16(address, kTab5InaRegDie, die, kTab5PowerIoMs);
      if (tab5_ina_verdict(answered, manufacturer, die) != Tab5InaVerdict::Ina226) continue;
      address_ = address;
      ready_ = true;
      io_.print(kTab5PowerEstimateLog);
      return true;
    }
    io_.print(kTab5PowerAbsentLog);
    return false;
  }

  // One poll: one triggered shunt+bus conversion (the config write starts it and
  // the chip powers down by itself when it is done), one fixed wait, one bus
  // register read. Nothing is polled, nothing keeps converting between polls.
  // A failed transfer returns nullopt -- never the previous reading -- and the
  // next poll starts over with a fresh conversion.
  std::optional<Tab5BatteryReading> read() {
    if (!ready_) return std::nullopt;
    if (!io_.write16(address_, kTab5InaRegConfig, kTab5InaConfigOneShot, kTab5PowerIoMs)) return failed();
    io_.delay(kTab5InaConversionWaitMs);
    uint16_t raw = 0;
    if (!io_.read16(address_, kTab5InaRegBus, raw, kTab5PowerIoMs)) return failed();
    if (failed_) {
      failed_ = false;
      io_.print(kTab5PowerRecoveredLog);
    }
    Tab5BatteryReading reading;
    reading.mv = tab5_ina_bus_mv(raw);
    reading.present = tab5_pack_present(reading.mv);
    // A percentage only for a fitted pack: 1213 mV with the cells removed would
    // clamp to 0% and read as a flat battery instead of a missing one.
    if (reading.present) reading.percent = tab5_pack_percent(reading.mv);
    return reading;
  }

 private:
  // One log line per failure transition, not one per poll.
  std::optional<Tab5BatteryReading> failed() {
    if (!failed_) {
      failed_ = true;
      io_.print(kTab5PowerFailedLog);
    }
    return std::nullopt;
  }

  IO& io_;
  uint8_t address_ = 0;
  bool ready_ = false;
  bool failed_ = false;
};

}  // namespace hgp
