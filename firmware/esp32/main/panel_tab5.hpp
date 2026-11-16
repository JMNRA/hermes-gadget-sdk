// Original ILI9881C/GT911 Tab5 only. Hardware facts from Espressif's
// m5stack_tab5 1.3.1, esp-bsp 73ee07b1ad56f865f13a2739be8bb6808c1da58a.
#pragma once
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
}  // namespace hgp
