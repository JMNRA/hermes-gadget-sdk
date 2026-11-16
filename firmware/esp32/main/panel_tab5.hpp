// Original ILI9881C/GT911 Tab5 only. Hardware facts from Espressif's
// m5stack_tab5 1.3.1, esp-bsp 73ee07b1ad56f865f13a2739be8bb6808c1da58a.
#pragma once
#include "board.hpp"

namespace hgp {
inline BoardConfig tab5_config(const char* name) {
  BoardConfig b{};
  b.name = name;
  b.radio_enabled = false;
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
  return b;
}

// BSP_POWER_AMP_IO is GPIO_NUM_NC on this board: the amplifier enable is the
// 0x43 expander's pin 1 (see tab5_prepare_speaker_off).

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
