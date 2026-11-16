// Interactive Tab5 audio diagnostic: the ESP adapter behind tab5_audio_ui.hpp.
// Opt-in build mode (CONFIG_HG_TAB5_AUDIO_UI), never part of a default, offline
// App, preview, microphone or speaker diagnostic build.
//
// This mode exists so the user can hear and speak while watching the panel:
// the screen from the confirmed preview diagnostic stays up, every tap on the
// top half plays one short tone through the reviewed speaker state machine, and
// the bottom half measures a short microphone window. The amplifier enable
// convention is no longer guessed: on this unit the control driven HIGH was
// audible and driven LOW was silent, so HIGH is the default path. Isolation
// rules for this file:
//
//   - One duplex I2S_NUM_1 standard channel (MCLK30/BCLK27/WS29, DOUT26, DIN28)
//     with both handles, two 16-bit slots: MIC1|MIC2 from the ES7210 and L|R to
//     the ES8388. It is the only I2S channel this build creates.
//   - The 0x43 expander's pin 1 (amplifier enable) is driven only through
//     tab5_speaker_diagnostic.hpp's masked read-modify-writes with readback: the
//     display/touch gates ran first, through their own device handle, and that
//     handle is gone before audio starts. Every write here preserves their
//     LCD/touch bits, and no BSP expander driver or cache is instantiated.
//   - The ES7210 opens on a microphone tap and is closed the moment the window
//     bound passes. No samples are stored, streamed, logged or persisted.
//   - No hg::App, NVS, identity, network, OTA, console or host seed exchange.
#include "port.hpp"  // first: pulls in FreeRTOS.h ahead of task.h/queue.h

#if CONFIG_HG_TAB5_AUDIO_UI

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "panel_tab5.hpp"
#include "tab5_audio_ui.hpp"

namespace hgp {

// main.cpp: the 0x43 display/touch gates and the GT911 probe, exactly the
// sequence the confirmed preview diagnostic runs. Declared here instead of in
// the header so the host tests never see an ESP-IDF type.
bool tab5_display_touch_gate(i2c_master_bus_handle_t bus);

namespace {

const char* TAG = "hg.audio-ui";
constexpr Tab5MicFormat kFormat = tab5_mic_format();
// ~10.7 ms per read at 24 kHz: the last read of a window overshoots by less
// than one block.
constexpr size_t kFramesPerRead = 256;
// The tone is padded to the bus's two slots in this many frames per write.
constexpr size_t kToneChunk = 512;

uint64_t clock_ms() { return static_cast<uint64_t>(esp_timer_get_time() / 1000); }

// The buses, both codecs and the amplifier expander. Every method the speaker
// state machine calls is the IO contract documented in
// tab5_speaker_diagnostic.hpp; the microphone window uses open_adc/read_adc/
// close_adc only.
struct Audio {
  i2s_chan_handle_t tx = nullptr, rx = nullptr;
  i2c_master_dev_handle_t expander = nullptr;
  esp_codec_dev_handle_t dac = nullptr;  // ES8388, created unopened, opened per tone
  esp_codec_dev_handle_t adc = nullptr;  // ES7210, opened only inside a window
  bool dac_open = false;
  bool adc_open = false;

  uint64_t now_ms() const { return clock_ms(); }

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
  // DAC only, two 16-bit slots, volume and mute applied before anything plays.
  bool open_dac(uint8_t volume, bool muted, uint32_t timeout) {
    (void)timeout;  // esp_codec_dev's own I2C writes are bounded inside the driver
    if (!dac || dac_open) return false;
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = kTab5SpeakerRate;
    fs.channel = kFormat.channels;  // the bus's two slots, duplicated below
    fs.bits_per_sample = 16;
    if (esp_codec_dev_open(dac, &fs) != ESP_CODEC_DEV_OK) {
      ESP_LOGE(TAG, "the ES8388 did not start");
      return false;
    }
    dac_open = true;
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
    (void)timeout;  // one codec register write
    if (!dac) return false;
    if (esp_codec_dev_set_out_mute(dac, muted) != ESP_CODEC_DEV_OK) {
      ESP_LOGW(TAG, "the ES8388 did not accept mute=%d", int(muted));
      return false;
    }
    return true;
  }
  // One mono block from the state machine, duplicated into the bus's two slots
  // and written TX only. Success means queued, never that the speaker played it.
  bool tone(const int16_t* pcm, size_t frames, uint32_t timeout) {
    (void)timeout;  // esp_codec_dev's own 1000 ms I2S wait bounds this call
    if (!dac || !pcm || !frames) return false;
    static int16_t stereo[kToneChunk * 2];
    size_t done = 0;
    while (done < frames) {
      const size_t chunk = std::min(kToneChunk, frames - done);
      for (size_t i = 0; i < chunk; ++i) {
        stereo[i * 2] = pcm[done + i];
        stereo[i * 2 + 1] = pcm[done + i];
      }
      const int bytes = static_cast<int>(chunk * 2 * sizeof(int16_t));
      if (esp_codec_dev_write(dac, stereo, bytes) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "the tone was not accepted by the DAC");
        return false;
      }
      done += chunk;
    }
    return true;
  }
  // Closes the ES8388 so the DAC is silent between taps. The device handle stays
  // alive for the next tap: esp_codec_dev 1.5 has no delete for its codec,
  // control or data interfaces, and this mode must be able to play again without
  // rebooting. Nothing is left open after a tone: the codec is closed every time.
  bool close_dac(uint32_t timeout) {
    (void)timeout;  // closes only what was opened
    if (!dac || !dac_open) return true;  // never opened: there is nothing to release
    esp_codec_dev_close(dac);
    dac_open = false;
    return true;
  }
  // ES7210, MIC1|MIC2, opened only for the window the screen started.
  bool start_adc() {
    if (!adc || adc_open) return false;
    esp_codec_dev_sample_info_t fs = {};
    fs.sample_rate = kFormat.sample_rate;
    fs.channel = kFormat.channels;
    fs.bits_per_sample = kFormat.bits;
    if (esp_codec_dev_open(adc, &fs) != ESP_CODEC_DEV_OK) {
      ESP_LOGE(TAG, "the ES7210 did not start");
      return false;
    }
    adc_open = true;
    // The gain and the selected microphones follow the SDK's own codec defaults
    // instead of a second copy. Which slot carries which microphone and the
    // exact gain are still unquantified; the probe averages both into mono.
    const CodecAudioConfig defaults{};
    if (esp_codec_dev_set_in_gain(adc, defaults.mic_gain_db) != ESP_CODEC_DEV_OK)
      ESP_LOGW(TAG, "mic gain was not accepted; the codec keeps its own default");
    return true;
  }
  // The adapter cannot report a short read: a timeout leaves the previous block
  // in place, exactly as the shipped CodecMic path behaves.
  bool read_adc(int16_t* interleaved, size_t frames) {
    if (!adc || !adc_open) return false;
    const int bytes = static_cast<int>(frames * kFormat.channels * sizeof(int16_t));
    return esp_codec_dev_read(adc, interleaved, bytes) == ESP_CODEC_DEV_OK;
  }
  void stop_adc() {
    if (adc && adc_open) {
      esp_codec_dev_close(adc);
      adc_open = false;
    }
  }
};

// The board's shared I2C bus plus this mode's own 0x43 device handle. The
// display/touch gate has already run and released its handle: from here on this
// mode is the only owner of the expander, and every write is masked.
bool start_expander(Audio& audio) {
  i2c_master_bus_handle_t bus = i2c::bus(board_config().i2c);
  if (!bus) {
    ESP_LOGE(TAG, "I2C bus failed; the amplifier enable cannot be driven");
    return false;
  }
  i2c_device_config_t device = {};
  device.device_address = kTab5SpeakerExpanderAddr;
  device.scl_speed_hz = board_config().i2c.hz;
  if (i2c_master_bus_add_device(bus, &device, &audio.expander) != ESP_OK) {
    ESP_LOGE(TAG, "the 0x%02x expander address was not accepted", kTab5SpeakerExpanderAddr);
    return false;
  }
  return true;
}

// The duplex I2S bus, the ES8388 (OUT) and the ES7210 (IN) device handles. Both
// codec devices stay unopened: the tone opens the DAC on its tap, and the
// microphone opens and closes its ADC on every window.
bool start_codecs(Audio& audio, std::string& fault) {
  i2c_master_bus_handle_t bus = i2c::bus(board_config().i2c);
  if (!bus) {
    fault = "I2C bus failed";
    return false;
  }
  const Tab5MicPins mic = tab5_mic_pins();
  const Tab5SpeakerPins spk = tab5_speaker_pins();
  i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  chan.auto_clear = true;  // silence on underrun instead of repeating the last block
  if (i2s_new_channel(&chan, &audio.tx, &audio.rx) != ESP_OK) {
    fault = "I2S_NUM_1 duplex channel failed";
    return false;
  }
  i2s_std_config_t cfg = {};
  cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kFormat.sample_rate);  // MCLK = 256 x fs
  cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  cfg.gpio_cfg.mclk = static_cast<gpio_num_t>(mic.mclk);
  cfg.gpio_cfg.bclk = static_cast<gpio_num_t>(mic.bclk);
  cfg.gpio_cfg.ws = static_cast<gpio_num_t>(mic.ws);
  cfg.gpio_cfg.din = static_cast<gpio_num_t>(mic.din);
  cfg.gpio_cfg.dout = static_cast<gpio_num_t>(spk.dout);
  // Both handles take the same clock and slot configuration: they share WS.
  if (i2s_channel_init_std_mode(audio.tx, &cfg) != ESP_OK ||
      i2s_channel_init_std_mode(audio.rx, &cfg) != ESP_OK) {
    fault = "I2S transmit/receive pin map or clock failed";
    return false;
  }
  audio_codec_i2s_cfg_t i2s_cfg = {};
  i2s_cfg.port = I2S_NUM_1;
  i2s_cfg.rx_handle = audio.rx;
  i2s_cfg.tx_handle = audio.tx;
  const audio_codec_data_if_t* data_if = audio_codec_new_i2s_data(&i2s_cfg);
  if (!data_if) {
    fault = "I2S data interface failed";
    return false;
  }
  const CodecAudioConfig defaults{};
  const audio_codec_gpio_if_t* gpio_if = audio_codec_new_gpio();
  if (!gpio_if) {
    fault = "codec GPIO interface failed";
    return false;
  }
  audio_codec_i2c_cfg_t dac_i2c = {};
  dac_i2c.port = I2C_NUM_0;
  // The device header's own address (8-bit form; the I2C driver shifts it).
  dac_i2c.addr = ES8388_CODEC_DEFAULT_ADDR;
  dac_i2c.bus_handle = bus;
  es8388_codec_cfg_t dac_cfg = {};
  dac_cfg.ctrl_if = audio_codec_new_i2c_ctrl(&dac_i2c);
  dac_cfg.gpio_if = gpio_if;
  dac_cfg.codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC;  // DAC only: the ADC is the ES7210
  dac_cfg.pa_pin = -1;  // the amplifier is the expander's pin 1, never a GPIO
  dac_cfg.pa_reverted = false;
  dac_cfg.master_mode = false;  // the P4 is the I2S master
  dac_cfg.hw_gain.pa_voltage = defaults.amp_supply_v;
  dac_cfg.hw_gain.codec_dac_voltage = 3.3;
  const audio_codec_if_t* dac_if = dac_cfg.ctrl_if ? es8388_codec_new(&dac_cfg) : nullptr;
  if (!dac_if) {
    fault = "ES8388 driver failed";
    return false;
  }
  esp_codec_dev_cfg_t out = {};
  out.dev_type = ESP_CODEC_DEV_TYPE_OUT;
  out.codec_if = dac_if;
  out.data_if = data_if;
  audio.dac = esp_codec_dev_new(&out);
  if (!audio.dac) {
    fault = "ES8388 device handle failed";
    return false;
  }
  audio_codec_i2c_cfg_t adc_i2c = {};
  adc_i2c.port = I2C_NUM_0;
  adc_i2c.addr = ES7210_CODEC_DEFAULT_ADDR;
  adc_i2c.bus_handle = bus;
  es7210_codec_cfg_t adc_cfg = {};
  adc_cfg.ctrl_if = audio_codec_new_i2c_ctrl(&adc_i2c);
  adc_cfg.master_mode = false;
  adc_cfg.mic_selected = defaults.es7210_mics;
  const audio_codec_if_t* adc_if = adc_cfg.ctrl_if ? es7210_codec_new(&adc_cfg) : nullptr;
  if (!adc_if) {
    fault = "ES7210 driver failed";
    return false;
  }
  esp_codec_dev_cfg_t in = {};
  in.dev_type = ESP_CODEC_DEV_TYPE_IN;
  in.codec_if = adc_if;
  in.data_if = data_if;
  audio.adc = esp_codec_dev_new(&in);
  if (!audio.adc) {
    fault = "ES7210 device handle failed";
    return false;
  }
  return true;
}

// One tap, one tone: the reviewed speaker state machine prepares the ES8388
// muted (the amplifier control is already in its reviewed off state), drives the
// amplifier control HIGH - the convention measured on this unit, where HIGH was
// audible and LOW was silent - writes the single 250 ms tone, mutes, requests the
// reviewed off state and verifies it by readback, then closes the DAC. Nothing
// plays without a tap, and the screen's own state machine owns the cooldown, so a
// refused tap never reaches this function.
void play_one_tone(Audio& audio, Tab5AudioScreen& screen, hg::Ui& ui, uint64_t now_ms) {
  Tab5SpeakerDiagnostic<Audio> diagnostic(audio);
  const bool prepared = diagnostic.prepare(kTab5AudioVolume);
  const bool played = prepared && diagnostic.play_once(true, kTab5AudioTonePeak);
  screen.tone_done(diagnostic.state(), now_ms);
  ui.render(screen.model());
  std::printf("audio ui: tone %u with amplifier %s; PA requested off and read back, state %s\n",
              screen.tones_played(), tab5_audio_pa_text(kTab5AudioConfirmedPa),
              tab5_speaker_state_name(diagnostic.state()));
  if (!played) {
    ESP_LOGW(TAG, "the tone sequence failed after %u tone(s); the state on screen is %s",
             screen.tones_played(), tab5_speaker_state_name(diagnostic.state()));
  }
}

[[noreturn]] void halt() {
  // No reboot, retry or fallback to other firmware: an unattended retry could
  // re-open a codec or the amplifier in an unknown state.
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

}  // namespace

[[noreturn]] void tab5_audio_ui_run() {
  std::printf(
      "audio ui: Tab5 interactive audio, duplex I2S %u Hz 2x16-bit, ES8388 tone + ES7210 mic; "
      "no app, NVS, identity, network, OTA or storage\n",
      unsigned(kFormat.sample_rate));
  std::printf("audio ui: top-half taps play one %u ms tone at volume %u, about -12 dBFS, "
              "amplifier %s (confirmed on this unit); bottom-half taps open a %u ms mic window\n",
              unsigned(kTab5SpeakerToneMs), unsigned(kTab5AudioVolume),
              tab5_audio_pa_text(kTab5AudioConfirmedPa), unsigned(kTab5AudioMicWindowMs));
  const auto& board = board_config();
  auto bus = i2c::bus(board.i2c);
  if (!tab5_display_touch_gate(bus)) {
    ESP_LOGE(TAG, "display/touch gates or GT911 probe failed; no panel to show the controls on");
    halt();
  }
  static DsiDisplay display;
  if (!display.begin(board.lcd, board.lcd.dsi) || !display.framebuffer()) {
    ESP_LOGE(TAG, "display initialization failed; nothing can be shown");
    halt();
  }
  hg::Ui ui(display);
  static Tab5AudioScreen screen(board.lcd.width, board.lcd.height);
  display.set_backlight(50);
  // The controls go up before any audio work: a panel that lights and says what
  // failed beats a black screen with a serial log nobody is reading.
  ui.render(screen.model());
  const bool events_ready = events::init();  // no touch producer or receive on a null queue
  static TouchInput touch;
  const bool input_ready = events_ready && touch.begin(board.touch, {}, {}, bus);
  if (!input_ready) {
    // Both halves are reached by touch only: without the event queue or the
    // GT911 there is no way to start either one, so say so on the panel and
    // touch no other device.
    screen.set_input_fault(events_ready ? "touch unavailable" : "the event queue failed");
    ui.render(screen.model());
    ESP_LOGE(TAG, "no touch input; neither half can be reached");
    halt();
  }

  Audio audio;
  std::string fault;
  Tab5MicStats stats;
  int16_t interleaved[kFramesPerRead * kFormat.channels];
  int16_t mono[kFramesPerRead];
  uint64_t window_start = 0, last_report = 0;
  if (!start_expander(audio) || !start_codecs(audio, fault)) {
    if (fault.empty()) fault = "the 0x43 expander did not answer";
    screen.set_audio_fault(fault);
    ui.render(screen.model());
    ESP_LOGE(TAG, "audio path unavailable: %s", fault.c_str());
  } else {
    ESP_LOGI(TAG, "audio ui ready: tap the top half for one tone, the bottom half for a mic window");
  }

  for (;;) {
    // The held status line moves on by itself, so a result or a return to ready
    // is never flashed past faster than it can be read.
    if (screen.tick(clock_ms())) ui.render(screen.model());
    // A microphone window reads one block per pass; events are still drained so
    // the screen keeps up. The bound closes the ADC, never a keystroke.
    if (screen.mic_active()) {
      if (!tab5_audio_mic_window_open(window_start, clock_ms())) {
        audio.stop_adc();  // the ES7210 is closed the moment the window ends
        screen.mic_done(stats);
        ui.render(screen.model());
        std::printf("audio ui: mic window closed at its %u ms bound\n",
                    unsigned(kTab5AudioMicWindowMs));
      } else if (!audio.read_adc(interleaved, kFramesPerRead)) {
        vTaskDelay(pdMS_TO_TICKS(10));
      } else {
        const size_t frames = tab5_mic_to_mono(interleaved, kFramesPerRead, kFormat.channels, mono,
                                               kFramesPerRead);
        if (frames) {
          stats.add(mono, frames);
          if (tab5_mic_report_due(clock_ms(), last_report) &&
              screen.mic_level(hg::level_percent(hg::rms(mono, frames))))
            ui.render(screen.model());
        }
      }
    }
    hgp::Event event{};
    // While capturing, never block: the window is what bounds the ADC.
    if (!events::receive(event, pdMS_TO_TICKS(screen.mic_active() ? 0 : 100))) continue;
    do {
      if (event.type == hgp::EventType::Touch && event.data &&
          event.len == sizeof(hgp::TouchSample)) {
        hgp::TouchSample sample{};
        std::memcpy(&sample, event.data, sizeof(sample));
        const Tab5AudioAction action = screen.touch(sample.touching, sample.x, sample.y, clock_ms());
        ui.render(screen.model());
        if (action == Tab5AudioAction::PlayTone) {
          play_one_tone(audio, screen, ui, clock_ms());
        } else if (action == Tab5AudioAction::MeasureMic) {
          stats = Tab5MicStats{};
          if (!audio.start_adc()) {
            screen.set_mic_fault("the ES7210 did not start");
            ui.render(screen.model());
          } else {
            window_start = clock_ms();
            last_report = window_start;
            std::printf("audio ui: mic window open for %u ms\n",
                        unsigned(kTab5AudioMicWindowMs));
          }
        }
      }
      hgp::events::release(event);
    } while (hgp::events::receive(event, 0));
  }
}

}  // namespace hgp

#endif  // CONFIG_HG_TAB5_AUDIO_UI
