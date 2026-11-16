// Microphone-only Tab5 diagnostic: the ESP adapter behind
// tab5_mic_diagnostic.hpp. Opt-in build mode (CONFIG_HG_TAB5_MIC_OFFLINE), never
// part of a default or offline-App build.
//
// Isolation rules for this file: RX-only I2S on I2S_NUM_1 with no TX pin at all,
// the ES7210 opened through esp_codec_dev only while capturing, and nothing
// else. It never opens the ES8388 DAC, never touches the amplifier or the 0x43
// expander, never calls the BSP audio helpers or bsp_feature_enable, and never
// constructs hg::App, NVS, network, OTA, an identity or the display.
#include "port.hpp"  // first: pulls in FreeRTOS.h ahead of task.h/queue.h

#if CONFIG_HG_TAB5_MIC_OFFLINE

#include <cstdio>
#include <cstring>

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "panel_tab5.hpp"
#include "tab5_mic_diagnostic.hpp"

namespace hgp {
namespace {

const char* TAG = "hg.mic";
constexpr Tab5MicFormat kFormat = tab5_mic_format();
// ~10.7 ms per read at 24 kHz: small enough that the last read of the session
// overshoots the 30 s bound by less than one block.
constexpr size_t kFramesPerRead = 256;

uint64_t now_ms() { return static_cast<uint64_t>(esp_timer_get_time() / 1000); }

struct Capture {
  i2s_chan_handle_t rx = nullptr;
  esp_codec_dev_handle_t adc = nullptr;
};

// One RX-only I2S channel plus the ES7210 on the shared I2C bus. `tx_handle`
// stays null and `dout` unused, so the probe cannot transmit on the bus.
bool start_capture(Capture& capture) {
  const Tab5MicPins pins = tab5_mic_pins();
  i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chan.auto_clear = true;  // silence on underrun instead of repeating the last block
  if (i2s_new_channel(&chan, nullptr, &capture.rx) != ESP_OK) {
    ESP_LOGE(TAG, "I2S_NUM_1 receive channel failed");
    return false;
  }
  i2s_std_config_t cfg = {};
  cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kFormat.sample_rate);  // MCLK = 256 x fs
  cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      kFormat.bits == 32 ? I2S_DATA_BIT_WIDTH_32BIT : I2S_DATA_BIT_WIDTH_16BIT,
      kFormat.channels == 2 ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO);
  cfg.gpio_cfg.mclk = static_cast<gpio_num_t>(pins.mclk);
  cfg.gpio_cfg.bclk = static_cast<gpio_num_t>(pins.bclk);
  cfg.gpio_cfg.ws = static_cast<gpio_num_t>(pins.ws);
  cfg.gpio_cfg.din = static_cast<gpio_num_t>(pins.din);
  cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;  // the probe never drives DOUT (GPIO 26)
  if (i2s_channel_init_std_mode(capture.rx, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "I2S receive pin map or clock failed");
    return false;
  }
  // The channel stays disabled until esp_codec_dev_open() starts it.

  audio_codec_i2s_cfg_t i2s_cfg = {};
  i2s_cfg.port = I2S_NUM_1;
  i2s_cfg.rx_handle = capture.rx;
  i2s_cfg.tx_handle = nullptr;
  const audio_codec_data_if_t* data_if = audio_codec_new_i2s_data(&i2s_cfg);
  if (!data_if) {
    ESP_LOGE(TAG, "I2S data interface failed");
    return false;
  }
  // The board's shared I2C bus, created on first use: no expander write, no
  // display or touch gate, no BSP constructor.
  i2c_master_bus_handle_t bus = i2c::bus(board_config().i2c);
  if (!bus) {
    ESP_LOGE(TAG, "I2C bus failed; the ES7210 cannot be configured");
    return false;
  }
  audio_codec_i2c_cfg_t i2c_cfg = {};
  i2c_cfg.port = I2C_NUM_0;
  // The device header's own address (8-bit form; the I2C driver shifts it).
  i2c_cfg.addr = ES7210_CODEC_DEFAULT_ADDR;
  i2c_cfg.bus_handle = bus;
  const audio_codec_ctrl_if_t* ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
  if (!ctrl_if) {
    ESP_LOGE(TAG, "ES7210 I2C control interface failed");
    return false;
  }
  // MIC1|MIC2 and the gain follow the SDK's own codec defaults instead of a
  // second copy: which slot carries which microphone is still unverified, and
  // the probe averages both into mono.
  const CodecAudioConfig defaults{};
  es7210_codec_cfg_t adc_cfg = {};
  adc_cfg.ctrl_if = ctrl_if;
  adc_cfg.master_mode = false;  // the P4 is the I2S master
  adc_cfg.mic_selected = defaults.es7210_mics;
  const audio_codec_if_t* codec_if = es7210_codec_new(&adc_cfg);
  if (!codec_if) {
    ESP_LOGE(TAG, "ES7210 driver failed");
    return false;
  }
  esp_codec_dev_cfg_t dev_cfg = {};
  dev_cfg.dev_type = ESP_CODEC_DEV_TYPE_IN;
  dev_cfg.codec_if = codec_if;
  dev_cfg.data_if = data_if;
  capture.adc = esp_codec_dev_new(&dev_cfg);
  if (!capture.adc) {
    ESP_LOGE(TAG, "ES7210 device handle failed");
    return false;
  }
  esp_codec_dev_sample_info_t fs = {};
  fs.sample_rate = kFormat.sample_rate;
  fs.channel = kFormat.channels;
  fs.bits_per_sample = kFormat.bits;
  if (esp_codec_dev_open(capture.adc, &fs) != ESP_CODEC_DEV_OK) {
    ESP_LOGE(TAG, "ES7210 did not start");
    return false;
  }
  if (esp_codec_dev_set_in_gain(capture.adc, defaults.mic_gain_db) != ESP_CODEC_DEV_OK)
    ESP_LOGW(TAG, "mic gain was not accepted; the codec keeps its own default");
  return true;
}

// Releases the ADC and then the I2S channel, in that order. esp_codec_dev 1.5
// has no delete for its codec/control/data interfaces, so those allocations
// stay: this mode runs once per boot and never re-opens them.
void stop_capture(Capture& capture) {
  if (capture.adc) {
    esp_codec_dev_close(capture.adc);
    esp_codec_dev_delete(capture.adc);
    capture.adc = nullptr;
  }
  if (capture.rx) {
    i2s_channel_disable(capture.rx);
    i2s_del_channel(capture.rx);
    capture.rx = nullptr;
  }
}

[[noreturn]] void print_summary(const char* reason, const Tab5MicStats& stats, unsigned failures,
                                unsigned unprintable) {
  char line[128];
  if (tab5_mic_summary_line(line, sizeof(line), stats))
    std::printf("%s\n", line);
  else
    ESP_LOGW(TAG, "the summary line did not fit its buffer");
  std::printf("mic probe: %s; %u unprintable lines, %u read failures\n", reason, unprintable,
              failures);
  // One shot per boot: no retry, no second session, no fallback to other audio.
  std::printf("mic probe: ADC closed; a new boot or reset is required to run it again\n");
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

}  // namespace

[[noreturn]] void tab5_mic_diagnostic_run() {
  std::printf(
      "mic probe: RX-only I2S_NUM_1 %u Hz, %u slots of %u bits, no DAC, no amplifier, "
      "no expander, no app\n",
      unsigned(kFormat.sample_rate), unsigned(kFormat.channels), unsigned(kFormat.bits));
  const Tab5MicPins pins = tab5_mic_pins();
  std::printf("mic probe: MCLK %d BCLK %d WS %d DIN %d (no DOUT)\n", pins.mclk, pins.bclk,
              pins.ws, pins.din);
  std::printf("mic probe: session bounded to %u ms, one line every %u ms\n",
              unsigned(kTab5MicSessionMs), unsigned(kTab5MicReportMs));

  Capture capture;
  if (!start_capture(capture)) {
    stop_capture(capture);
    // Nothing was captured: the summary reports an empty session, which is the
    // honest result of a codec that did not start.
    print_summary("the ES7210 did not start", Tab5MicStats{}, 0, 0);
  }
  std::printf("mic probe: ES7210 open; speak or play a tone at the tablet\n");

  const uint64_t start = now_ms();
  uint64_t last_report = start;
  Tab5MicStats stats;
  unsigned failures = 0, unprintable = 0;
  int16_t interleaved[kFramesPerRead * kFormat.channels];
  int16_t mono[kFramesPerRead];
  while (tab5_mic_session_open(start, now_ms())) {
    // The adapter cannot report a short read: a timeout leaves the previous
    // block in place, exactly as the shipped CodecMic path behaves.
    if (esp_codec_dev_read(capture.adc, interleaved, sizeof(interleaved)) != ESP_CODEC_DEV_OK) {
      ++failures;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    const size_t frames =
        tab5_mic_to_mono(interleaved, kFramesPerRead, kFormat.channels, mono, kFramesPerRead);
    if (!frames) continue;
    stats.add(mono, frames);
    if (!tab5_mic_report_due(now_ms(), last_report)) continue;
    // The same level math the App's VAD uses, so the numbers are comparable.
    char line[64];
    if (tab5_mic_level_line(line, sizeof(line), hg::level_percent(hg::rms(mono, frames))))
      std::printf("%s\n", line);
    else
      ++unprintable;
  }
  const uint64_t elapsed = now_ms() - start;
  stop_capture(capture);
  std::printf("mic probe: session ended after %u ms\n", unsigned(elapsed));
  print_summary("the session reached its bound", stats, failures, unprintable);
}

}  // namespace hgp

#endif  // CONFIG_HG_TAB5_MIC_OFFLINE
