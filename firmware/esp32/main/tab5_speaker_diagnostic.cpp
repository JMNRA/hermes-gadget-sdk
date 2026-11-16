// Speaker-only Tab5 diagnostic: the ESP adapter behind
// tab5_speaker_diagnostic.hpp. Opt-in build mode (CONFIG_HG_TAB5_SPEAKER_OFFLINE),
// never part of a default, offline-App, microphone or display-diagnostic build.
//
// Isolation rules for this file: the ES8388 as a DAC on a TX-only I2S channel
// (I2S_NUM_1, MCLK30/BCLK27/WS29/DOUT26), and the amplifier enable on the 0x43
// expander's pin 1 through masked read-modify-writes that are read back. This
// file is the exclusive owner of that expander: it never instantiates or resets
// the BSP expander driver, and never calls the BSP audio helpers or
// bsp_feature_enable. It never opens the ES7210 or any ADC, never stores or
// streams audio, and never constructs hg::App, NVS, network, OTA, an identity,
// the display, or the host seed exchange.
#include "port.hpp"  // first: pulls in FreeRTOS.h ahead of task.h/queue.h

#if CONFIG_HG_TAB5_SPEAKER_OFFLINE

#include <cstdio>

#include "driver/usb_serial_jtag.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "panel_tab5.hpp"
#include "tab5_speaker_diagnostic.hpp"

namespace hgp {
namespace {

const char* TAG = "hg.spk";
// Nothing plays at boot: the operator asks for the one tone over USB
// Serial/JTAG, and an absent or refused start leaves the amplifier off.
constexpr uint32_t kStartWindowMs = 30000;

struct Speaker {
  i2s_chan_handle_t tx = nullptr;
  esp_codec_dev_handle_t dac = nullptr;      // created unopened, opened once
  i2c_master_dev_handle_t expander = nullptr;

  uint64_t now_ms() const { return static_cast<uint64_t>(esp_timer_get_time() / 1000); }

  bool read(uint8_t reg, uint8_t& value, uint32_t timeout) {
    const esp_err_t err = i2c_master_transmit_receive(expander, &reg, 1, &value, 1, timeout);
    if (err != ESP_OK) ESP_LOGW(TAG, "expander read 0x%02x failed: %s", reg, esp_err_to_name(err));
    return err == ESP_OK;
  }
  bool write(uint8_t reg, uint8_t value, uint32_t timeout) {
    const uint8_t data[] = {reg, value};
    const esp_err_t err = i2c_master_transmit(expander, data, sizeof(data), timeout);
    if (err != ESP_OK) ESP_LOGW(TAG, "expander write 0x%02x failed: %s", reg, esp_err_to_name(err));
    return err == ESP_OK;
  }
  // DAC only: esp_codec_dev_open starts the I2S TX clock and configures the
  // ES8388 in DAC mode (the driver's own open sequence), then the volume is
  // applied and the output muted. The ES7210 is never created on this bus.
  bool open_dac(uint8_t volume, bool muted, uint32_t timeout) {
    (void)timeout;  // the codec calls are bounded internally (100 ms per I2C write)
    if (!dac) return false;
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = kTab5SpeakerRate;
    fs.channel = 1;  // one 16-bit slot, as every SDK codec board runs
    fs.bits_per_sample = 16;
    if (esp_codec_dev_open(dac, &fs) != ESP_CODEC_DEV_OK) {
      ESP_LOGE(TAG, "the ES8388 did not start");
      return false;
    }
    if (esp_codec_dev_set_out_vol(dac, volume) != ESP_CODEC_DEV_OK) {
      ESP_LOGE(TAG, "the ES8388 did not accept volume %u", unsigned(volume));
      return false;
    }
    if (esp_codec_dev_set_out_mute(dac, muted) != ESP_CODEC_DEV_OK) {
      ESP_LOGE(TAG, "the ES8388 did not accept the mute request");
      return false;
    }
    return true;
  }
  bool set_mute(bool muted, uint32_t timeout) {
    (void)timeout;  // one codec register write (100 ms I2C timeout inside the driver)
    if (!dac) return false;
    if (esp_codec_dev_set_out_mute(dac, muted) != ESP_CODEC_DEV_OK) {
      ESP_LOGW(TAG, "the ES8388 did not accept mute=%d", int(muted));
      return false;
    }
    return true;
  }
  // One block, TX only. esp_codec_dev_write returns once the block was queued
  // (its I2S write waits at most 1000 ms): this is "written", not "heard".
  bool tone(const int16_t* pcm, size_t frames, uint32_t timeout) {
    (void)timeout;  // esp_codec_dev's own 1000 ms I2S wait bounds this call
    if (!dac || !pcm || !frames) return false;
    const int bytes = static_cast<int>(frames * sizeof(int16_t));
    if (esp_codec_dev_write(dac, const_cast<int16_t*>(pcm), bytes) != ESP_CODEC_DEV_OK) {
      ESP_LOGE(TAG, "the tone was not accepted by the DAC");
      return false;
    }
    return true;
  }
  // Releases the codec handle. esp_codec_dev 1.5 has no delete for its codec,
  // control or data interfaces, so those allocations stay: this mode runs once
  // per boot and never re-opens them. i2s_channel_disable in teardown waits for
  // the DMA to drain and has no timeout, so it is not bounded here.
  bool close_dac(uint32_t timeout) {
    (void)timeout;  // closes only what was opened; failures of the chip are reported by readback
    if (!dac) return true;  // never opened: there is nothing to release
    esp_codec_dev_close(dac);
    esp_codec_dev_delete(dac);
    dac = nullptr;
    return true;
  }
};

// The board's shared I2C bus (SDA31/SCL32) plus our own 0x43 device handle. No
// expander reset, no BSP cache, no display/touch gate: this build owns it.
bool start_expander(Speaker& out) {
  i2c_master_bus_handle_t bus = i2c::bus(board_config().i2c);
  if (!bus) {
    ESP_LOGE(TAG, "I2C bus failed; the amplifier enable cannot be driven");
    return false;
  }
  i2c_device_config_t device = {};
  device.device_address = kTab5SpeakerExpanderAddr;
  device.scl_speed_hz = board_config().i2c.hz;
  if (i2c_master_bus_add_device(bus, &device, &out.expander) != ESP_OK) {
    ESP_LOGE(TAG, "the 0x%02x expander address was not accepted", kTab5SpeakerExpanderAddr);
    return false;
  }
  return true;
}

// TX-only I2S plus the ES8388 device handle. The receive handle stays null and
// DIN is unused, so this build cannot capture anything.
bool start_output(Speaker& out) {
  const Tab5SpeakerPins pins = tab5_speaker_pins();
  i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chan.auto_clear = true;  // silence on underrun instead of repeating the last block
  if (i2s_new_channel(&chan, &out.tx, nullptr) != ESP_OK) {
    ESP_LOGE(TAG, "I2S_NUM_1 transmit channel failed");
    return false;
  }
  i2s_std_config_t cfg = {};
  cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kTab5SpeakerRate);  // MCLK = 256 x fs
  cfg.slot_cfg =
      I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  cfg.gpio_cfg.mclk = static_cast<gpio_num_t>(pins.mclk);
  cfg.gpio_cfg.bclk = static_cast<gpio_num_t>(pins.bclk);
  cfg.gpio_cfg.ws = static_cast<gpio_num_t>(pins.ws);
  cfg.gpio_cfg.dout = static_cast<gpio_num_t>(pins.dout);
  cfg.gpio_cfg.din = I2S_GPIO_UNUSED;  // the probe never reads DIN (GPIO 28)
  if (i2s_channel_init_std_mode(out.tx, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "I2S transmit pin map or clock failed");
    return false;
  }
  // The channel stays disabled until esp_codec_dev_open() starts it.

  audio_codec_i2s_cfg_t i2s_cfg = {};
  i2s_cfg.port = I2S_NUM_1;
  i2s_cfg.rx_handle = nullptr;
  i2s_cfg.tx_handle = out.tx;
  const audio_codec_data_if_t* data_if = audio_codec_new_i2s_data(&i2s_cfg);
  if (!data_if) {
    ESP_LOGE(TAG, "I2S data interface failed");
    return false;
  }
  i2c_master_bus_handle_t bus = i2c::bus(board_config().i2c);
  if (!bus) {
    ESP_LOGE(TAG, "I2C bus failed; the ES8388 cannot be configured");
    return false;
  }
  audio_codec_i2c_cfg_t i2c_cfg = {};
  i2c_cfg.port = I2C_NUM_0;
  // The device header's own address (8-bit form; the I2C driver shifts it).
  i2c_cfg.addr = ES8388_CODEC_DEFAULT_ADDR;
  i2c_cfg.bus_handle = bus;
  const audio_codec_ctrl_if_t* ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
  if (!ctrl_if) {
    ESP_LOGE(TAG, "ES8388 I2C control interface failed");
    return false;
  }
  const audio_codec_gpio_if_t* gpio_if = audio_codec_new_gpio();
  if (!gpio_if) {
    ESP_LOGE(TAG, "codec GPIO interface failed");
    return false;
  }
  // The hardware gain follows the SDK's own codec defaults instead of a second
  // copy of the numbers; the actual Tab5 amplifier supply is still unverified.
  const CodecAudioConfig defaults{};
  es8388_codec_cfg_t dac_cfg = {};
  dac_cfg.ctrl_if = ctrl_if;
  dac_cfg.gpio_if = gpio_if;
  dac_cfg.codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC;  // DAC only: no ADC path
  dac_cfg.pa_pin = -1;  // the amplifier is the expander's pin 1, never a GPIO
  dac_cfg.pa_reverted = false;
  dac_cfg.master_mode = false;  // the P4 is the I2S master
  dac_cfg.hw_gain.pa_voltage = defaults.amp_supply_v;
  dac_cfg.hw_gain.codec_dac_voltage = 3.3;
  const audio_codec_if_t* codec_if = es8388_codec_new(&dac_cfg);
  if (!codec_if) {
    ESP_LOGE(TAG, "ES8388 driver failed");
    return false;
  }
  esp_codec_dev_cfg_t dev_cfg = {};
  dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_OUT;
  dev_cfg.codec_if = codec_if;
  dev_cfg.data_if = data_if;
  out.dac = esp_codec_dev_new(&dev_cfg);
  if (!out.dac) {
    ESP_LOGE(TAG, "ES8388 device handle failed");
    return false;
  }
  return true;
}

// Bounded wait for the deliberate start. Reading one byte at a time keeps the
// window honest: the first CR/LF is skipped, 's'/'S' starts, anything else
// refuses the tone. USB Serial/JTAG ingress loss is not observable here, and no
// byte is treated as data or persisted.
Tab5SpeakerStart await_start() {
  usb_serial_jtag_driver_config_t usb = {};
  usb.tx_buffer_size = 256;
  usb.rx_buffer_size = 256;
  if (usb_serial_jtag_is_driver_installed() ||
      usb_serial_jtag_driver_install(&usb) != ESP_OK) {
    ESP_LOGE(TAG, "speaker probe: USB Serial/JTAG input is unavailable");
    return Tab5SpeakerStart::Refuse;
  }
  std::printf("speaker probe: send 's' over USB Serial/JTAG within %u ms for the single tone\n",
              unsigned(kStartWindowMs));
  const uint64_t start = static_cast<uint64_t>(esp_timer_get_time() / 1000);
  while (true) {
    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time() / 1000);
    if (now < start || now - start >= kStartWindowMs) break;  // a backwards clock also closes it
    uint8_t byte = 0;
    if (usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(100)) != 1) continue;
    const Tab5SpeakerStart action = tab5_speaker_start_byte(byte);
    if (action == Tab5SpeakerStart::Ignore) continue;
    return action;
  }
  return Tab5SpeakerStart::Refuse;
}

// Deletes the TX channel, states the final state, then halts. Deleting the
// channel is not bounded (i2s_channel_disable waits for the DMA to drain), so
// the result line is printed before it: an event that hangs teardown cannot
// hide the only evidence of what was or was not verified.
[[noreturn]] void finish(Speaker& out, Tab5SpeakerState state, const char* reason) {
  char line[64];
  if (tab5_speaker_result_line(line, sizeof(line), state))
    std::printf("%s\n", line);
  else
    ESP_LOGW(TAG, "the result line did not fit its buffer");
  if (state != Tab5SpeakerState::SafeOff)
    std::printf("speaker probe: the amplifier state is Unknown; do not assume it is off\n");
  if (out.tx) {
    i2s_channel_disable(out.tx);
    i2s_del_channel(out.tx);
    out.tx = nullptr;
  }
  std::printf("speaker probe: %s; ES8388 closed, I2S TX channel deleted\n", reason);
  // One attempt per boot: no retry, no second tone, no fallback.
  std::printf("speaker probe: one attempt per boot; a new boot or reset is required to run again\n");
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

}  // namespace

[[noreturn]] void tab5_speaker_diagnostic_run() {
  std::printf(
      "speaker probe: DAC-only ES8388 on I2S_NUM_1 %u Hz, mono 16-bit; no ADC, no ES7210, "
      "no display, no app\n",
      unsigned(kTab5SpeakerRate));
  const Tab5SpeakerPins pins = tab5_speaker_pins();
  std::printf("speaker probe: MCLK %d BCLK %d WS %d DOUT %d (no DIN), volume %u, one %u Hz tone "
              "for %u ms at about -40 dBFS\n",
              pins.mclk, pins.bclk, pins.ws, pins.dout, unsigned(kTab5SpeakerVolume),
              unsigned(kTab5SpeakerToneHz), unsigned(kTab5SpeakerToneMs));

  Speaker out;
  if (!start_expander(out) || !start_output(out)) {
    // Nothing was opened: report the honest state and stop without a tone.
    Tab5SpeakerDiagnostic<Speaker> diagnostic(out);
    finish(out, diagnostic.state(), "the output path did not start");
  }
  std::printf("speaker probe: expander 0x%02x is owned exclusively; no BSP driver, no reset\n",
              unsigned(kTab5SpeakerExpanderAddr));
  Tab5SpeakerDiagnostic<Speaker> diagnostic(out);
  if (!diagnostic.prepare()) {
    std::printf("speaker probe: the ES8388 or the amplifier request failed\n");
    finish(out, diagnostic.state(), "preparation failed");
  }
  std::printf("speaker probe: pa off requested and read back (0x%02x pin %u, masked)\n",
              unsigned(kTab5SpeakerExpanderAddr), unsigned(kTab5SpeakerPaPin));
  std::printf("speaker probe: es8388 open (DAC only, volume %u, muted)\n",
              unsigned(kTab5SpeakerVolume));

  const Tab5SpeakerStart start = await_start();
  if (start != Tab5SpeakerStart::Start) {
    // Nothing played: leave the amplifier off and report what was verified.
    std::printf("speaker probe: no explicit start; no tone written\n");
    const bool disarmed = diagnostic.disarm();
    finish(out, diagnostic.state(), disarmed ? "disarmed without a tone" : "disarm failed");
  }
  std::printf("speaker probe: pa on requested and read back\n");
  if (!diagnostic.play_once(true)) {
    std::printf("speaker probe: phase failed; the amplifier state is reported below\n");
    finish(out, diagnostic.state(), "the tone attempt failed");
  }
  std::printf("speaker probe: tone written (queued to the DAC, not proof of sound)\n");
  std::printf("speaker probe: pa off requested and read back\n");
  finish(out, diagnostic.state(), "the single tone completed");
}

}  // namespace hgp

#endif  // CONFIG_HG_TAB5_SPEAKER_OFFLINE
