// Hermes Gadget firmware entry point: wires the ESP32 drivers to the portable
// core (hg::App) and runs the app loop on the main task.
#include "port.hpp"  // first: pulls in FreeRTOS.h ahead of task.h/queue.h
#include "port_cardputer.hpp"
#include "port_sdlog.hpp"

#include <cstring>
#include <utility>
#include <vector>

#include "driver/gpio.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "freertos/task.h"
#include "hg/touch.hpp"
#include "nvs_flash.h"
#include "sdkconfig.h"
#if CONFIG_HG_TAB5_APP_OFFLINE
// Explicitly isolated diagnostic only: USB loss is not fully observable and
// historical exchanges may replay. Host-controlled ephemeral identity is NOT
// production authentication or evidence of hardware entropy.
#include "tab5_offline_app.hpp"
// The confirmed offline preview UI and its touch feedback, reused here without
// the preview diagnostic build mode: the offline App never waits on a host with
// a black panel.
#include "tab5_diagnostic.hpp"
#include "esp_random.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#if !CONFIG_HG_BOARD_M5STACK_TAB5 || CONFIG_HG_TAB5_DIAGNOSTIC
#error "Tab5 offline App requires Tab5 and excludes the preview diagnostic"
#endif
#endif
#if CONFIG_HG_TAB5_DIAGNOSTIC
#include "tab5_diagnostic.hpp"
#if !CONFIG_HG_BOARD_M5STACK_TAB5
#error "The display/touch diagnostic is only supported on Tab5"
#endif
#endif
#if CONFIG_HG_TAB5_MIC_OFFLINE
// One bounded microphone probe: no App, NVS, identity, network, OTA, display or
// speaker path, and no audio retention. It releases the ADC and never returns.
#include "tab5_mic_diagnostic.hpp"
#if !CONFIG_HG_BOARD_M5STACK_TAB5 || CONFIG_HG_TAB5_DIAGNOSTIC || CONFIG_HG_TAB5_APP_OFFLINE
#error "The microphone diagnostic is a Tab5-only, mutually exclusive startup mode"
#endif
#endif
#if CONFIG_HG_TAB5_SPEAKER_OFFLINE
// One bounded speaker probe: no App, NVS, identity, network, OTA, display or
// microphone path. The single tone needs a deliberate start, the amplifier is
// requested off and read back, and the mode never returns.
#include "tab5_speaker_diagnostic.hpp"
#if !CONFIG_HG_BOARD_M5STACK_TAB5 || CONFIG_HG_TAB5_DIAGNOSTIC || CONFIG_HG_TAB5_APP_OFFLINE || \
    CONFIG_HG_TAB5_MIC_OFFLINE
#error "The speaker diagnostic is a Tab5-only, mutually exclusive startup mode"
#endif
#endif
#if CONFIG_HG_TAB5_AUDIO_UI
// Interactive audio probe: the preview screen and touch stay live while the top
// half plays one tone per tap and the bottom half measures a short microphone
// window. No App, NVS, identity, network, OTA, storage or host seed exchange.
#include "tab5_audio_ui.hpp"
#if !CONFIG_HG_BOARD_M5STACK_TAB5 || CONFIG_HG_TAB5_DIAGNOSTIC || CONFIG_HG_TAB5_APP_OFFLINE || \
    CONFIG_HG_TAB5_MIC_OFFLINE || CONFIG_HG_TAB5_SPEAKER_OFFLINE
#error "The interactive audio diagnostic is a Tab5-only, mutually exclusive startup mode"
#endif
#endif
#if CONFIG_HG_BOARD_M5STACK_TAB5
#include "panel_tab5.hpp"
#endif
#if CONFIG_HG_BOARD_WS_P4_TOUCH_LCD_7B || CONFIG_HG_BOARD_CORES3
#include "hg/crypto.hpp"
#include "port_camera.hpp"
#endif
#if SOC_MIPI_DSI_SUPPORTED
#include "esp_ldo_regulator.h"
#endif

namespace {
const char* TAG = "hg.main";
}  // namespace

namespace hgp {
#if CONFIG_HG_BOARD_M5STACK_TAB5
// The 0x43 display/touch gates and the GT911 probe the confirmed preview
// diagnostic runs. Defined once here, never duplicated, so every Tab5
// diagnostic mode brings the panel up with the same sequence
// (tab5_audio_ui.cpp calls it as well).
bool tab5_display_touch_gate(i2c_master_bus_handle_t bus) {
  if (!bus) return false;
  i2c_device_config_t config{};
  config.device_address = 0x43;
  config.scl_speed_hz = 400000;
  i2c_master_dev_handle_t dev = nullptr;
  if (i2c_master_bus_add_device(bus, &config, &dev) != ESP_OK) return false;
  const bool ready = hgp::tab5_enable_display_touch(
      [&](uint8_t reg, uint8_t& value) {
        return i2c_master_transmit_receive(dev, &reg, 1, &value, 1, 100) == ESP_OK;
      },
      [&](uint8_t reg, uint8_t value) {
        const uint8_t data[] = {reg, value};
        return i2c_master_transmit(dev, data, sizeof(data), 100) == ESP_OK;
      },
      [&] {
        vTaskDelay(pdMS_TO_TICKS(500) + 1);
        // Identify the original touch before driving a pin shared by later revisions.
        if (i2c_master_probe(bus, 0x14, 100) != ESP_OK) return false;
        // BSP v1 workaround: hold GT911 INT low while polling the touch chip.
        gpio_config_t pin{};
        pin.pin_bit_mask = 1ULL << 23;
        pin.mode = GPIO_MODE_OUTPUT;
        pin.pull_up_en = GPIO_PULLUP_ENABLE;
        if (gpio_config(&pin) != ESP_OK || gpio_set_level(GPIO_NUM_23, 0) != ESP_OK) {
          if (gpio_reset_pin(GPIO_NUM_23) != ESP_OK)
            ESP_LOGE(TAG, "Tab5 touch INT could not be released after setup failure");
          return false;
        }
        return true;
      });
  i2c_master_bus_rm_device(dev);  // only our device handle, never the shared bus
  return ready;
}
#endif
}  // namespace hgp

namespace {

#if SOC_MIPI_DSI_SUPPORTED
esp_ldo_channel_handle_t g_p4_vdd_io_ldo = nullptr;

bool enable_p4_vdd_io_rail() {
  esp_ldo_channel_config_t config{};
  config.chan_id = 4;
  config.voltage_mv = 3300;
  const esp_err_t err = esp_ldo_acquire_channel(&config, &g_p4_vdd_io_ldo);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "VDD_IO_5 LDO4 at 3300 mV failed: %s", esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "VDD_IO_5 enabled by LDO4 at 3300 mV");
  return true;
}
#endif

#if CONFIG_HG_TAB5_APP_OFFLINE
[[noreturn]] void stop_tab5_offline() {
  // No reboot, retry, or normal-startup fallback. The host must use a fresh seed;
  // repeated device challenges still permit an intact historical transcript.
  for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

// Best-effort, bounded host seed exchange. Returns true only when the verified
// exchange handed a complete seed over inside the window. Every other outcome is
// a plain "no host seed": the caller keeps the preview running.
bool try_tab5_offline_seed(hgp::Tab5OfflineSeed& seed, hgp::Tab5OfflineExchange& exchange,
                           const std::array<uint8_t, 16>& challenge, uint64_t window_start,
                           std::array<uint8_t, 56>& running) {
  auto now = [] { return static_cast<uint64_t>(esp_timer_get_time()/1000); };
  usb_serial_jtag_driver_config_t usb{};
  usb.tx_buffer_size = 256;
  usb.rx_buffer_size = 256;
  if (usb_serial_jtag_is_driver_installed() ||
      usb_serial_jtag_driver_install(&usb) != ESP_OK) {
    ESP_LOGW(TAG, "offline: USB Serial/JTAG unavailable");
    return false;
  }
  uint8_t extra = 0;
  // Reject buffered ingress, never flush evidence. Earlier hardware/FIFO drops
  // remain unobservable with this driver. Proof/ACK rejects observed mismatches,
  // but does not establish lossless ingress, freshness or authentication.
  if (usb_serial_jtag_read_bytes(&extra, 1, 0) != 0) {
    hgp::tab5_wipe(&extra, 1);
    ESP_LOGW(TAG, "offline: USB ingress was not empty");
    return false;
  }
  uint8_t ready[24]{};
  std::memcpy(ready, "HGREADY3", 8);
  std::memcpy(ready + 8, challenge.data(), challenge.size());
  if (usb_serial_jtag_write_bytes(ready, sizeof(ready), pdMS_TO_TICKS(1000)) != sizeof(ready) ||
      usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(1000)) != ESP_OK) {
    ESP_LOGW(TAG, "offline: READY frame was not accepted by USB");
    return false;
  }
  while (hgp::tab5_offline_window_open(window_start, now())) {
    uint8_t bytes[57]{}; // one surplus byte makes an oversized frame fail closed
    const int n = usb_serial_jtag_read_bytes(bytes, sizeof(bytes), pdMS_TO_TICKS(100));
    const bool valid = n >= 0 && exchange.feed(bytes, static_cast<size_t>(n), now());
    hgp::tab5_wipe(bytes, sizeof(bytes));
    if (!valid) {
      ESP_LOGW(TAG, "offline: host seed frame rejected");
      return false;
    }
    if (exchange.awaiting_proof()) {
      uint8_t proof[56]{};
      const bool sent = exchange.response(proof, sizeof(proof), now()) &&
          usb_serial_jtag_write_bytes(proof, sizeof(proof), pdMS_TO_TICKS(1000)) == sizeof(proof) &&
          usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(1000)) == ESP_OK;
      std::memcpy(running.data(), proof, running.size());
      std::memcpy(running.data(), "HGRUN002", 8);
      hgp::tab5_wipe(proof, sizeof(proof));
      if (!sent) {
        ESP_LOGW(TAG, "offline: proof frame was not accepted by USB");
        return false;
      }
    }
    if (exchange.accepted()) break;
  }
  if (!exchange.accepted()) {
    ESP_LOGW(TAG, "offline: no host seed within %u ms",
             static_cast<unsigned>(hgp::kTab5OfflineSeedWindowMs));
    return false;
  }
  if (usb_serial_jtag_read_bytes(&extra, 1, pdMS_TO_TICKS(250)) != 0) {
    hgp::tab5_wipe(&extra, 1);
    ESP_LOGW(TAG, "offline: unexpected bytes after the ACK");
    return false;
  }
  if (!exchange.finish(seed, now())) {
    ESP_LOGW(TAG, "offline: seed handover outside the window");
    return false;
  }
  return true;
}

// The fallback UI: the same confirmed preview the diagnostic build shows, with
// its coordinate feedback. No App, identity, NVS, network or audio is created.
[[noreturn]] void run_tab5_offline_preview(hg::Ui& ui, hg::UiModel& model, bool events_ready) {
  ESP_LOGW(TAG, "offline preview only: no App, identity, NVS, network or audio");
  for (;;) {
    if (!events_ready) { // no event queue: keep the panel up and stay quiet
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    hgp::Event event{};
    if (!hgp::events::receive(event, pdMS_TO_TICKS(100))) continue;
    if (event.type == hgp::EventType::Touch && event.data &&
        event.len == sizeof(hgp::TouchSample)) {
      hgp::TouchSample sample{};
      std::memcpy(&sample, event.data, sizeof(sample));
      ESP_LOGI(TAG, "preview touch %s x=%d y=%d", sample.touching ? "down/move" : "up",
               static_cast<int>(sample.x), static_cast<int>(sample.y));
      hgp::tab5_diagnostic_touch(model, sample.touching, sample.x, sample.y);
      ui.render(model);
    }
    hgp::events::release(event);
  }
}

void run_tab5_offline_app() {
  const auto& board = hgp::board_config();
  auto bus = hgp::i2c::bus(board.i2c);
  if (!tab5_display_touch_gate(bus)) {
    ESP_LOGE(TAG, "offline: display/touch gates or original GT911 probe failed");
    stop_tab5_offline();
  }
  static hgp::DsiDisplay display;
  if (!display.begin(board.lcd, board.lcd.dsi) || !display.framebuffer()) {
    ESP_LOGE(TAG, "offline: display initialization failed");
    stop_tab5_offline();
  }
  // The confirmed preview goes up before anything waits on USB: a reboot with no
  // host seed must show this UI, never a black panel.
  hg::Ui ui(display);
  hg::UiModel preview = hgp::tab5_diagnostic_preview();
  display.set_backlight(50); // the panel starts dark; App owns this on its route
  ui.render(preview);
  ESP_LOGI(TAG, "offline preview rendered; host seed window %u ms",
           static_cast<unsigned>(hgp::kTab5OfflineSeedWindowMs));
  const bool events_ready = hgp::events::init(); // no touch producer or receive on a null queue
  static hgp::TouchInput touch;
  if (!events_ready || !touch.begin(board.touch, {}, {}, bus))
    ESP_LOGE(TAG, "offline: touch unavailable; the preview keeps running without input");

  hgp::Tab5OfflineSeed seed;
  auto now = [] { return static_cast<uint64_t>(esp_timer_get_time()/1000); };
  // Best-effort replay discriminator ONLY, not cryptographic entropy. With
  // radios/entropy disabled it may repeat after reset. Never seed keys from it.
  std::array<uint8_t,16> challenge{};
  esp_fill_random(challenge.data(), challenge.size());
  const uint64_t timer = static_cast<uint64_t>(esp_timer_get_time());
  for (size_t i=0; i<8; ++i) challenge[i] ^= static_cast<uint8_t>(timer >> (8*i));
  const uint64_t window_start = now();
  hgp::Tab5OfflineExchange exchange(window_start, challenge);
  std::array<uint8_t,56> running{};
  const bool seeded = try_tab5_offline_seed(seed, exchange, challenge, window_start, running);
  if (hgp::tab5_offline_route(seeded,
                              hgp::tab5_offline_window_open(window_start, now())) !=
      hgp::Tab5OfflineRoute::App) {
    // No host seed: keep the preview and its touch feedback. Nothing persistent
    // was created, and any seed bytes already handed over are wiped here.
    exchange.cancel();
    seed.feed(nullptr, 1);
    hgp::tab5_wipe(running.data(), running.size());
    run_tab5_offline_preview(ui, preview, events_ready);
  }
  uint8_t extra = 0;
  auto stop = [&] {
    hgp::tab5_wipe(running.data(), running.size());
    exchange.cancel(); seed.feed(nullptr, 1); stop_tab5_offline();
  };
  if (!events_ready) stop(); // the App loop dispatches touch through that queue
  hgp::Tab5OfflineSystem system(seed, [] { return static_cast<uint32_t>(esp_timer_get_time()/1000); },
                                stop_tab5_offline);
  hgp::Tab5OfflineTransport transport;
  hgp::Tab5OfflineStorage storage; // volatile settings only; no NVS or identity retention
  auto hal = hgp::tab5_offline_hal(system, transport, display, storage);
  hg::App app(hal, hgp::tab5_offline_profile(board.touch.ppi));
  hg::TouchGestures gestures(app);
  if (usb_serial_jtag_read_bytes(&extra, 1, 0) != 0) {
    hgp::tab5_wipe(&extra, 1);
    stop();
  }
  app.begin();
  if (!system.consumed() || transport.violated()) stop();
  if (usb_serial_jtag_write_bytes(running.data(), running.size(), pdMS_TO_TICKS(1000)) != sizeof(running) ||
      usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(1000)) != ESP_OK) stop();
  hgp::tab5_wipe(running.data(), running.size());
  for (;;) {
    // The USB session is one-shot: even a late second frame is a violation.
    if (usb_serial_jtag_read_bytes(&extra, 1, 0) != 0) { hgp::tab5_wipe(&extra, 1); stop(); }
    hgp::Event event{};
    if (hgp::events::receive(event, pdMS_TO_TICKS(10))) {
      const bool valid = event.type == hgp::EventType::Touch && event.data &&
                         event.len == sizeof(hgp::TouchSample);
      if (valid) {
        hgp::TouchSample sample{};
        std::memcpy(&sample, event.data, sizeof(sample));
        gestures.update(sample.touching, sample.x, sample.y, system.now_ms());
      }
      hgp::events::release(event);
      if (!valid) stop(); // never dispatch network, console, audio, or auth events
    }
    gestures.tick(system.now_ms());
    app.tick();
    if (transport.violated()) stop();
  }
}
#elif CONFIG_HG_TAB5_DIAGNOSTIC
// No full-firmware globals or startup are instantiated in this mode.
void run_tab5_diagnostic() {
  ESP_LOGI(TAG, "TAB5 DISPLAY/TOUCH DIAGNOSTIC: no app, storage, keys, network or audio startup");
  const auto& board = hgp::board_config();
  auto bus = hgp::i2c::bus(board.i2c);
  if (!tab5_display_touch_gate(bus)) {
    ESP_LOGE(TAG, "diagnostic stopped: display/touch gates or GT911 probe failed");
    return;
  }
  static hgp::DsiDisplay display;
  if (!display.begin(board.lcd, board.lcd.dsi) ||
      !hgp::tab5_diagnostic_pattern(display.framebuffer(), board.lcd.width, board.lcd.height)) {
    ESP_LOGE(TAG, "diagnostic stopped: display initialization failed");
    return;
  }
  display.flush(0, board.lcd.height);
  display.set_backlight(50);
  ESP_LOGI(TAG, "pattern submitted: white border, red/green/blue bars (not physical proof)");
  hgp::events::init();
  static hgp::TouchInput touch;
  if (!touch.begin(board.touch, {}, {}, bus)) {
    ESP_LOGE(TAG, "diagnostic stopped: touch initialization failed");
    return;
  }
  ESP_LOGI(TAG, "touch ready: reporting down/move/up coordinates over serial");
  // Keep the startup color check visible, then use the real portable renderer
  // directly. No App, settings actions, or services are attached to this model.
  vTaskDelay(pdMS_TO_TICKS(1500));
  hg::Ui ui(display);
  auto model = hgp::tab5_diagnostic_preview();
  ui.render(model);
  ESP_LOGI(TAG, "local Hermes UI preview submitted (offline; touch feedback only)");
  for (;;) {
    hgp::Event event{};
    if (!hgp::events::receive(event, pdMS_TO_TICKS(100))) continue;
    if (event.type == hgp::EventType::Touch && event.data && event.len == sizeof(hgp::TouchSample)) {
      hgp::TouchSample sample{};
      std::memcpy(&sample, event.data, sizeof(sample));
      ESP_LOGI(TAG, "touch %s x=%d y=%d", sample.touching ? "down/move" : "up",
               static_cast<int>(sample.x), static_cast<int>(sample.y));
      hgp::tab5_diagnostic_touch(model, sample.touching, sample.x, sample.y);
      ui.render(model);
    }
    hgp::events::release(event);
  }
}
#else
hgp::EspSystem g_system;
hgp::NvsStorage g_storage;
hgp::WsTransport g_transport;
hgp::SpiDisplay g_display;
#if CONFIG_HG_BOARD_CARDPUTER_ADV
hg::CardputerCompose g_compose(g_display);
hgp::CardputerKeyboard g_keyboard;
#endif
// The microphone object holds handles and no buffers, so the CYD uses it too:
// its PDM mic runs on I2S0, the one port with the PDM-to-PCM converter.
hgp::I2sMic g_mic;
#if !CONFIG_HG_BOARD_ESP32_CYD
// The CYD has no PSRAM: its 320x240 framebuffer takes all but a few KiB of
// internal RAM, so only the drivers a board profile can select are
// instantiated. Its own peripherals are wired in the CYD branch of app_main.
hgp::ParallelDisplay g_parallel;
hgp::RgbDisplay g_rgb;
hgp::AmoledDisplay g_amoled;
#if SOC_MIPI_DSI_SUPPORTED
hgp::DsiDisplay g_dsi;
hgp::P4I2sSpeaker g_p4_speaker;
hgp::Stc8Power g_stc8_power;
#endif
hgp::I2sSpeaker g_speaker;
hgp::CodecAudio g_codec;
hgp::CodecMic g_codec_mic;
hgp::CodecSpeaker g_codec_speaker;
hgp::OpusSpeakerDecoder g_opus;
hgp::TouchInput g_touch;
hgp::MotionInput g_motion;
hgp::AxpPower g_power;
hgp::LatchPower g_latch_power;
hgp::CoreS3Board g_cores3;
hgp::StickS3Board g_sticks3;
hgp::StopWatchBoard g_stopwatch;
hgp::PhoneLink g_phone;
hgp::TailnetLink g_tailnet;
#endif
hgp::Buttons g_buttons;
hgp::Wifi g_wifi;
hgp::SdLogger g_sdlog;
#if CONFIG_HG_BOARD_WS_P4_TOUCH_LCD_7B
hgp::CameraCapture g_camera;
#endif
hgp::TimePort g_time;
hgp::EspUpdater g_updater;
hg::TouchGestures* g_gestures = nullptr;

// touch_cancel: which inputs act as CANCEL on touch boards.
//   both (default)  swipe down on the screen, and the PWR key
//   swipe           only the swipe;  pwr  only the PWR key
bool g_swipe_cancel = true;
bool g_key_cancel = true;

void apply_touch_cancel() {
  auto v = g_storage.get("touch_cancel");
  std::string mode = v ? *v : "both";
  g_swipe_cancel = mode != "pwr";
  g_key_cancel = mode != "swipe";
  if (g_gestures) g_gestures->set_swipe_cancel(g_swipe_cancel);
}

// The name a phone sees: the `name` setting, as hg::App uses it.
#if !CONFIG_HG_BOARD_ESP32_CYD  // the CYD has no Bluetooth phone link
std::string device_name() {
  auto v = g_storage.get("name");
  return v && !v->empty() ? *v : std::string(CONFIG_HG_DEFAULT_NAME);
}
#endif

void init_nvs() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

// Example device action: a plain status LED the agent can switch with "led.set".
void add_status_led(hg::App& app, int gpio) {
  if (gpio < 0) return;
  gpio_reset_pin(static_cast<gpio_num_t>(gpio));
  gpio_set_direction(static_cast<gpio_num_t>(gpio), GPIO_MODE_OUTPUT);
  hg::Action led;
  led.name = "led.set";
  led.description = "Turn the gadget's status LED on or off.";
  hg::json::Value props = hg::json::Value::object();
  hg::json::Value color = hg::json::Value::object();
  color.set("type", "string").set("description", "'off' turns it off; any other value turns it on");
  props.set("color", color);
  hg::json::Value required = hg::json::Value::array();
  required.push("color");
  led.params.set("type", "object").set("properties", props).set("required", required);
  led.handler = [gpio](const hg::json::Value& args, hg::json::Value& result, std::string& error) {
    const std::string& c = args["color"].as_string();
    if (c.empty()) {
      error = "color is required";
      return false;
    }
    bool on = c != "off";
    gpio_set_level(static_cast<gpio_num_t>(gpio), on ? 1 : 0);
    result.set("on", on);
    return true;
  };
  app.add_action(std::move(led));
}

#if CONFIG_HG_BOARD_WS_P4_TOUCH_LCD_7B
void add_camera_action(hg::App& app, i2c_master_bus_handle_t sccb_bus) {
  hg::Action camera;
  camera.name = "camera.capture";
  camera.description = "Take a still photo with the attached OV5647 camera. The gateway saves it as a PNG and returns its path.";
  camera.params.set("type", "object").set("properties", hg::json::Value::object());
  camera.handler = [sccb_bus](const hg::json::Value&, hg::json::Value& result, std::string& error) {
    std::vector<uint8_t> pixels;
    uint16_t width = 0, height = 0;
    if (!g_camera.capture_rgb565(sccb_bus, pixels, width, height, error)) return false;
    if (pixels.size() > 640u * 512u * 2u) {
      error = "captured image exceeds the action-result limit";
      return false;
    }
    result.set("format", "rgb565le")
        .set("width", static_cast<unsigned>(width))
        .set("height", static_cast<unsigned>(height))
        .set("image_b64", hg::crypto::base64_encode(pixels.data(), pixels.size()));
    return true;
  };
  app.add_action(std::move(camera));
}
#elif CONFIG_HG_BOARD_CORES3
// The Camera HAL's start/stop bookkeeping. The pipeline starts on demand (the
// camera mode opening, or a capture) and stops when it leaves, or 60 s after
// the last capture — a feature most sessions won't use need not pay for PSRAM
// and sensor power the whole boot. A start that fails latches for the boot:
// the Camera item hides and captures report the latch.
static uint32_t camera_use_ms = 0;  // the last start or capture: the 60 s stop counts from it

static bool ensure_camera_run() {
  if (hgp::camera_hal_running()) return true;
  if (hgp::camera_hal_start_failed()) return false;
  if (!g_cores3.camera_power(true)) {
    // camera_power(true) puts the rails up before it pulses the expander, so a
    // mid-way failure can leave the ribbon powered: down as far as the bus
    // answers, then report the failed start either way.
    g_cores3.camera_power(false);
    g_cores3.camera_rails_off();
    return false;
  }
  if (!hgp::camera_hal_begin()) {
    // A start that got as far as powering up leaves the camera powered down.
    g_cores3.camera_power(false);
    g_cores3.camera_rails_off();
    return false;
  }
  camera_use_ms = xTaskGetTickCount();
  return true;
}

static void stop_camera() {
  hgp::camera_hal_stop();       // the driver's frame buffers go back to PSRAM
  g_cores3.camera_power(false); // the sensor is held in reset
  g_cores3.camera_rails_off();  // and the ribbon's supplies go off
}

void add_camera_action(hg::App& app, i2c_master_bus_handle_t sccb_bus) {
  static hgp::CameraCapture cores3_camera;
  hg::Action camera;
  camera.name = "camera.capture";
  camera.description = "Take a still photo with the GC0308 camera. The gateway saves it as a PNG and returns its path.";
  camera.params.set("type", "object").set("properties", hg::json::Value::object());
  camera.handler = [sccb_bus](const hg::json::Value&, hg::json::Value& result, std::string& error) {
    std::vector<uint8_t> pixels;
    uint16_t width = 0, height = 0;
    if (!hgp::camera_hal_running()) ensure_camera_run();
    if (!cores3_camera.capture_rgb565(sccb_bus, pixels, width, height, error)) return false;
    camera_use_ms = xTaskGetTickCount();  // a capture keeps the pipeline up for its 60 s
    if (pixels.size() > 640u * 512u * 2u) {
      error = "captured image exceeds the action-result limit";
      return false;
    }
    result.set("format", "rgb565le")
        .set("width", static_cast<unsigned>(width))
        .set("height", static_cast<unsigned>(height))
        .set("image_b64", hg::crypto::base64_encode(pixels.data(), pixels.size()));
    return true;
  };
  app.add_action(std::move(camera));
}
#endif

// Supplies switched straight from a GPIO go on before anything talks to the
// parts behind them.
void power_rails_on(const hgp::BoardConfig& b) {
  bool any = false;
  for (const hgp::PowerRail& rail : b.power_rails) {
    if (rail.gpio < 0) continue;
    const auto pin = static_cast<gpio_num_t>(rail.gpio);
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, rail.on_level ? 1 : 0);
    any = true;
  }
  // One tick more than the rounded-down count, so the wait is never shorter at 100 Hz.
  if (any && b.power_rails_settle_ms) vTaskDelay(pdMS_TO_TICKS(b.power_rails_settle_ms) + 1);
}

// Light sleep disables every GPIO it doesn't know about; these must keep their
// level so the panel, touch controller, amplifier and buses survive a doze.
void keep_pins_through_sleep(const hgp::BoardConfig& b) {
  // A latch-powered board must also keep its battery latch high, or light sleep cuts the supply.
  const int pins[] = {b.amoled.cs, b.amoled.sclk, b.amoled.d0, b.amoled.d1, b.amoled.d2, b.amoled.d3,
                      b.amoled.rst, b.lcd.cs, b.lcd.dc, b.lcd.rst, b.lcd.backlight, b.touch.rst,
                      b.touch.bus.sda, b.touch.bus.scl, b.codec.pa, b.i2c.sda, b.i2c.scl,
                      b.latch_power.enabled ? b.latch_power.enable : -1};
  for (int pin : pins)
    if (pin >= 0) gpio_sleep_sel_dis(static_cast<gpio_num_t>(pin));
  for (const hgp::PowerRail& rail : b.power_rails)
    if (rail.gpio >= 0) gpio_sleep_sel_dis(static_cast<gpio_num_t>(rail.gpio));
}

void dispatch(hg::App& app, hgp::Event& ev) {
  using hgp::EventType;
  const char* text = reinterpret_cast<const char*>(ev.data);
  // Frames from a connection the app has already abandoned are dropped.
  bool stale = (ev.type == EventType::WsOpen || ev.type == EventType::WsText || ev.type == EventType::WsBinary ||
                ev.type == EventType::WsClosed) &&
               ev.generation != g_transport.generation();
  if (stale) return;
  switch (ev.type) {
    case EventType::NetUp:
      g_wifi.connected(app);
      app.on_network(true, text ? text : "");
      g_time.set_network(true);
#if !CONFIG_HG_BOARD_ESP32_CYD
      g_tailnet.network_up();
#endif
      break;
    case EventType::NetDown:
      app.on_network(false, text ? text : "");
      g_time.set_network(false);
#if !CONFIG_HG_BOARD_ESP32_CYD
      g_tailnet.network_down();
#endif
      break;
    case EventType::WifiDisconnected:
      g_wifi.disconnected();
      app.on_network(false, text ? text : "");
      g_time.set_network(false);
#if !CONFIG_HG_BOARD_ESP32_CYD
      g_tailnet.network_down();
#endif
      break;
    case EventType::WifiStarted: g_wifi.reconfigure(); break;
    case EventType::WifiScanDone: g_wifi.scan_done(); break;
    case EventType::WifiProvision:
      if (ev.len == sizeof(hg::WifiCredentials)) g_wifi.provision(*reinterpret_cast<const hg::WifiCredentials*>(ev.data));
      break;
    case EventType::WsOpen: app.on_transport_open(); break;
    case EventType::WsText: app.on_transport_text(std::string_view(text, ev.len)); break;
    case EventType::WsBinary: app.on_transport_binary(ev.data, ev.len); break;
    case EventType::WsClosed: app.on_transport_closed(text ? text : "closed"); break;
    case EventType::Mic:
      app.on_mic_samples(reinterpret_cast<const int16_t*>(ev.data), ev.len / sizeof(int16_t));
      break;
    case EventType::Console:
      ev.console->answer(app.console(std::string_view(text ? text : "", ev.len)));
      break;
    case EventType::Touch:
      if (g_gestures && ev.len == sizeof(hgp::TouchSample)) {
        const auto* t = reinterpret_cast<const hgp::TouchSample*>(ev.data);
        g_gestures->update(t->touching, t->x, t->y, g_system.now_ms());
      }
      break;
    case EventType::Key:
      if (ev.len == sizeof(hgp::KeySample)) {
        const bool pressed = reinterpret_cast<const hgp::KeySample*>(ev.data)->pressed;
        // Not CANCEL with `touch_cancel swipe`, but still a key that lights a dark screen
        // (DeviceProfile::wake_key): with touch_wake off it may be the only way back.
        if (g_key_cancel) app.on_button(hg::Button::Cancel, pressed);
        else if (pressed) app.wake_display();
      }
      break;
    case EventType::Encoder:
      if (ev.len == sizeof(hgp::EncoderSample)) {
        // A detent is a press and a release, so turning keeps scrolling after
        // the first detent wakes the screen.
        const auto* encoder = reinterpret_cast<const hgp::EncoderSample*>(ev.data);
        const hg::Button button = encoder->direction > 0 ? hg::Button::Up : hg::Button::Down;
        app.on_button(button, true);
        app.on_button(button, false);
      }
      break;
    case EventType::Phone:
      if (ev.len == sizeof(hgp::PhoneSample)) {
        const auto* phone = reinterpret_cast<const hgp::PhoneSample*>(ev.data);
        switch (phone->kind) {
          case hgp::PhoneSample::Kind::Passkey: app.on_phone_passkey(phone->passkey); break;
          case hgp::PhoneSample::Kind::Paired: app.on_phone_paired(true); break;
          case hgp::PhoneSample::Kind::Failed: app.on_phone_paired(false); break;
          case hgp::PhoneSample::Kind::Status: app.on_phone_status(phone->paired, phone->connected); break;
        }
      }
      break;
    case EventType::TimeSynced:
      g_time.on_synced();  // writes the RTC from the now-set clock
      break;
  }
}

#endif  // CONFIG_HG_TAB5_DIAGNOSTIC
}  // namespace

/** Initialize the selected board and serialize App, input and display work on this task. */
extern "C" void app_main(void) {
#if CONFIG_HG_TAB5_APP_OFFLINE
  run_tab5_offline_app();
  stop_tab5_offline();
#elif CONFIG_HG_TAB5_DIAGNOSTIC
  run_tab5_diagnostic();
  // Failure never falls through to full firmware or retries peripheral setup.
#elif CONFIG_HG_TAB5_MIC_OFFLINE
  hgp::tab5_mic_diagnostic_run();  // bounded session, releases the ADC, no return
#elif CONFIG_HG_TAB5_SPEAKER_OFFLINE
  hgp::tab5_speaker_diagnostic_run();  // one tone after a deliberate start, no return
#elif CONFIG_HG_TAB5_AUDIO_UI
  hgp::tab5_audio_ui_run();  // screen, one tone per tap, bounded mic windows, no return
#else
#if CONFIG_HG_BOARD_M5STACK_TAB5
  hgp::tab5_rng_init();  // The SAR ADC must be exclusive during this early seed window.
#endif
  hgp::diag::begin();  // first, so `diag log` has the whole boot
  init_nvs();
  hgp::events::init();
  ESP_ERROR_CHECK(g_storage.begin() ? ESP_OK : ESP_FAIL);
  const hgp::BoardConfig& board = hgp::board_config();
  // SD log mirror: starts disabled and syncs to the stored sd_log setting
  // below (off by default), so no file appears until it is switched on.
  // Boards without a slot stay silent.
  g_sdlog.begin(board.sd, false);
  power_rails_on(board);
  // A board whose amplifier hangs off the chip's own DAC (the CYD's SC8002B on
  // GPIO26) gets three boot beeps: they show the amplifier and speaker work
  // before any reply is routed there (port_dac.cpp). Boards without a DAC
  // speaker do nothing. Blocks for about 0.6 s.
  if (board.speaker_dac >= 0) hgp::dac_beep(board.speaker_dac);
#if !CONFIG_HG_BOARD_ESP32_CYD
  const bool latch_power = board.latch_power.enabled && g_latch_power.begin(board.latch_power);
#endif
  const char* version = esp_app_get_description()->version;
  ESP_LOGI(TAG, "Hermes Gadget %s on %s", version, board.name);
#if CONFIG_SPIRAM
#if CONFIG_SPIRAM_MODE_OCT
  constexpr const char* kPsramMode = "octal";
#else
  constexpr const char* kPsramMode = "quad";
#endif
  if (!esp_psram_is_initialized()) {
    ESP_LOGE(TAG, "no PSRAM found: this firmware is built for a module with %s PSRAM, and the display may "
                  "not start without it (see the board's requirements in docs/hardware.md)",
             kPsramMode);
  }
#endif
#if CONFIG_PM_ENABLE
  // Boards that enable power management let the CPU drop to 80 MHz when idle;
  // drivers hold it up while they work. Light sleep only on boards that ask for
  // it, and then only while dozing: the app holds the chip awake otherwise
  // (EspSystem::enable_doze), so the display, touch, audio and USB keep working.
  esp_pm_config_t pm = {};
  pm.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
  pm.min_freq_mhz = 80;
  pm.light_sleep_enable = board.light_sleep && g_system.enable_doze();
  if (esp_pm_configure(&pm) != ESP_OK) ESP_LOGW(TAG, "power management unavailable; the CPU stays at full speed");
#endif
  g_updater.expect_board(board.name);
  g_updater.start();  // a new firmware on probation starts its clock now

  // Shared network/event infrastructure first. Tab5's display-only profile
  // returns before Wi-Fi initialization; it never starts the hosted transport.
  g_wifi.begin(g_storage);

  hg::Hal hal;
  hal.system = &g_system;
  hal.transport = &g_transport;
  hal.storage = &g_storage;
#if !CONFIG_HG_BOARD_ESP32_CYD
  if (latch_power) hal.power = &g_latch_power;
  if (latch_power && board.latch_power.usb_external_power)
    g_wifi.on_battery = [] { return g_latch_power.external_power() == std::optional<bool>(false); };
#endif
  if (g_updater.capacity()) hal.updater = &g_updater;
#if CONFIG_HG_BOARD_ESP32_CYD
  // The CYD's peripherals are the panel, the BOOT key and a PDM microphone on
  // the 4-pin header: no I2C bus, expander, codec, touch controller or PMIC to
  // bring up.
  if (board.lcd.enabled && g_display.begin(board.lcd, nullptr)) hal.display = &g_display;
  if (board.mic.enabled && g_mic.begin(board.mic)) {
    hal.mic = &g_mic;
  } else if (board.mic.enabled) {
    // The PDM mic's I2S0 DMA and task stack did not fit next to the reserved
    // framebuffer band: lower kCydFrameRows in port_display.cpp if this appears.
    ESP_LOGE(TAG, "microphone did not start: not enough internal RAM for its buffers");
  }
  // No I2C bus on this board; the timezone setting and network time still apply.
  g_time.begin(board.rtc, nullptr, g_storage);
  constexpr bool touch = false;
  constexpr bool imu = false;  // no I2C bus: no motion sensor to bring up
  hgp::diag::Parts parts;
  parts.display = hal.display ? g_display.controller_name() : "none";
  parts.mic = hal.mic == &g_mic ? (board.mic.pdm ? "pdm" : "i2s") : "none";
  parts.time = &g_time;
#else
  bool io_rail_ready = true;
#if SOC_MIPI_DSI_SUPPORTED
  if (board.lcd.bus.type == hgp::LcdBus::Type::Dsi && board.lcd.dsi.vdd_io_ldo4)
    io_rail_ready = enable_p4_vdd_io_rail();
#endif
  // GPIO45/46 belong to VDD_IO_5 on the P4; acquire LDO4 before creating I2C0.
  i2c_master_bus_handle_t i2c_bus = io_rail_ready ? hgp::i2c::bus(board.i2c) : nullptr;
#if CONFIG_HG_BOARD_M5STACK_TAB5
  io_rail_ready = io_rail_ready && hgp::tab5_display_touch_gate(i2c_bus);
  if (!io_rail_ready) ESP_LOGE(TAG, "Tab5 display/touch gates or original GT911 probe failed");
#endif
  const bool peripherals_ready = io_rail_ready && (!board.cores3 || g_cores3.begin(i2c_bus)) &&
      (!board.sticks3 || g_sticks3.begin(i2c_bus)) &&
      (!board.stopwatch || g_stopwatch.begin(i2c_bus)) &&
      (!board.expander_reset.mask || hgp::tca9554_reset(i2c_bus, board.expander_reset)) &&
      (!board.expander_levels.enabled || hgp::tca9554_levels_begin(i2c_bus, board.expander_levels));
  if (board.cores3 && peripherals_ready)
    g_display.board_backlight = [](uint8_t percent) { g_cores3.set_backlight(percent); };
  if (board.expander_levels.backlight && peripherals_ready)
    g_display.board_backlight = [](uint8_t percent) { hgp::tca9554_backlight(percent); };
  if (peripherals_ready && board.lcd.enabled) {
#if SOC_MIPI_DSI_SUPPORTED
    if (board.lcd.bus.type == hgp::LcdBus::Type::Dsi) {
      if (g_dsi.begin(board.lcd, board.lcd.dsi)) hal.display = &g_dsi;
    } else
#endif
    if (board.lcd.bus.type == hgp::LcdBus::Type::Rgb) {
      if (g_rgb.begin(board.lcd, i2c_bus)) hal.display = &g_rgb;
    } else if (board.lcd.bus.type == hgp::LcdBus::Type::I80) {
      if (g_parallel.begin(board.lcd, hgp::lcd_power_pin(board))) hal.display = &g_parallel;
    } else if (g_display.begin(board.lcd, i2c_bus)) {
      hal.display = &g_display;
    }
  } else if (board.amoled.enabled && g_amoled.begin(board.amoled)) {
    hal.display = &g_amoled;
  }
  if (board.mic.enabled && g_mic.begin(board.mic)) hal.mic = &g_mic;
#if SOC_MIPI_DSI_SUPPORTED
  if (board.p4_speaker.enabled) {
    if (g_p4_speaker.begin(board.speaker, board.p4_speaker)) hal.speaker = &g_p4_speaker;
  } else
#endif
  if (board.speaker.enabled && g_speaker.begin(board.speaker)) hal.speaker = &g_speaker;
#if CONFIG_HG_BOARD_CORES3
  // The CoreS3's GC0308: the camera HAL rides the panel once the display is up.
  // The pipeline starts on demand (the camera mode opening, or a capture) and
  // stops after, so the boot pays nothing for a feature most sessions skip.
  if (hal.display != nullptr) {
    hgp::camera_hal_set_display(hal.display);
    hal.camera = hgp::camera_hal();
  }
#endif
  if (board.axp2101 && g_power.begin(i2c_bus)) {
    hal.power = &g_power;
    if (board.axp_power_key) g_power.use_power_key();
    g_power.use_curves(board.battery_curve, board.battery_charge_curve);
    // Wi-Fi retries back off only on battery; unknown counts as plugged in.
    g_wifi.on_battery = [] { return g_power.external_power() == std::optional<bool>(false); };
  }
#if SOC_MIPI_DSI_SUPPORTED
  if (peripherals_ready && board.stc8_battery && g_stc8_power.begin(i2c_bus)) hal.power = &g_stc8_power;
#endif
  if (board.sticks3 && peripherals_ready) {  // peripherals_ready includes g_sticks3.begin()
    hal.power = &g_sticks3;
    g_wifi.on_battery = [] { return g_sticks3.external_power() == std::optional<bool>(false); };
  }
  if (board.stopwatch && peripherals_ready) {  // peripherals_ready includes g_stopwatch.begin()
    hal.power = &g_stopwatch;
    g_stopwatch.use_curves(board.battery_curve, board.battery_charge_curve);
    g_wifi.on_battery = [] { return g_stopwatch.external_power() == std::optional<bool>(false); };
  }
  const bool audio_power = peripherals_ready && (!board.axp_audio_supply || g_power.enable_audio_supply());
  if (!audio_power) ESP_LOGE(TAG, "audio supply unavailable");
  if (board.sticks3) g_codec_speaker.board_pa = [](bool on) { g_sticks3.set_amplifier(on); };
  if (board.stopwatch) g_codec_speaker.board_pa = [](bool on) { g_stopwatch.set_amplifier(on); };
  if (board.codec.enabled && audio_power && g_codec.begin(board.codec, i2c_bus)) {
    if (g_codec_mic.begin(g_codec.in(), board.codec.rmnm_mics, g_codec.mic_link())) hal.mic = &g_codec_mic;
    g_codec_speaker.es8311_fade_ramp = board.codec.dac_fade_ramp && board.codec.speaker == hgp::SpeakerCodec::Es8311;
    if (g_codec_speaker.begin(g_codec.out(), board.codec.stereo32, board.codec.speaker_pa ? board.codec.pa : -1)) hal.speaker = &g_codec_speaker;
    // Light-sleep boards power the DAC down while dozing, if the mics don't share it.
    if (board.light_sleep && g_codec.mic_link().switchable) g_codec_speaker.allow_power_down(g_codec.mic_link().format);
  }
  if (hal.speaker) hal.speaker_decoder = &g_opus;  // Opus replies when Hermes offers them
  const bool imu = peripherals_ready && board.imu.enabled && g_motion.begin(board.imu, i2c_bus);
  g_time.begin(board.rtc, i2c_bus, g_storage);
  const bool touch = peripherals_ready && (board.touch.enabled || board.pwr_key.enabled || board.encoder.a >= 0) &&
                     g_touch.begin(board.touch, board.pwr_key, board.encoder,
                                   board.touch.bus.sda >= 0 ? hgp::i2c::touch_bus(board.touch.bus) : i2c_bus);

  hgp::diag::Parts parts;
#if SOC_MIPI_DSI_SUPPORTED
  if (hal.display == &g_dsi) {
    parts.display = g_dsi.controller_name();
  } else
#endif
  {
    parts.display = hal.display == &g_display ? g_display.controller_name()
                    : hal.display == &g_parallel ? "st7789-i80"
                    : hal.display == &g_rgb ? g_rgb.controller_name()
                    : hal.display == &g_amoled ? "co5300"
                                              : "none";
  }
  parts.mic = hal.mic == &g_codec_mic ? (board.codec.mic == hgp::MicCodec::Es8311 ? "es8311" : "es7210")
              : hal.mic == &g_mic ? (board.mic.pdm ? "pdm" : "i2s") : "none";
#if SOC_MIPI_DSI_SUPPORTED
  if (hal.speaker == &g_p4_speaker) parts.speaker = "i2s";
  else
#endif
  parts.speaker = hal.speaker == &g_codec_speaker ?
      (board.codec.speaker == hgp::SpeakerCodec::Aw88298 ? "aw88298" :
       board.codec.speaker == hgp::SpeakerCodec::Es8388 ? "es8388" : "es8311") :
      hal.speaker == &g_speaker ? "i2s" : "none";
  parts.touch = touch && g_touch.has_touch();
  parts.key = touch && g_touch.has_key();
  parts.touch_input = touch ? &g_touch : nullptr;
  parts.i2c = i2c_bus;
  parts.imu = imu ? g_motion.chip_name() : "none";
  parts.rtc = g_time.rtc_name();
  parts.motion = imu ? &g_motion : nullptr;
  parts.time = &g_time;
#endif
#if CONFIG_HG_BOARD_CARDPUTER_ADV
  const bool keyboard = g_keyboard.begin(i2c_bus);
  if (hal.display == &g_display) hal.display = &g_compose;
  parts.key = keyboard;
#endif
  g_buttons.begin(board.buttons);
  hgp::diag::set_parts(parts);
  hgp::diag::log_boot_summary();

  hg::DeviceProfile profile;
  profile.board = board.name;
  profile.firmware = version;
#if CONFIG_HG_FRAME_STATS
  profile.frame_stats = true;
#endif
  profile.frame_budget_us = CONFIG_HG_FRAME_BUDGET_US;
  profile.frame_log_ms = CONFIG_HG_FRAME_LOG_S * 1000u;
  profile.default_name = CONFIG_HG_DEFAULT_NAME;
#if CONFIG_SPIRAM
  profile.faces = esp_psram_is_initialized();
  // From PSRAM only, after the framebuffer: without room there, one-colour faces.
  if (profile.faces)
    profile.colour_face_alloc = [](size_t n) { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); };
  // The recording buffer (2 MB for 60 s at 16 kHz, 3 MB at 24 kHz), PSRAM only, and only when
  // about 1 MB of PSRAM stays free after it (RecordingBuffer::leaves_headroom). Otherwise the board
  // streams as before, and a connection lost mid-recording ends the recording.
  if (esp_psram_is_initialized())
    profile.recording_alloc = [](size_t n) -> void* {
      const size_t free_bytes = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
      if (!hg::RecordingBuffer::leaves_headroom(free_bytes, n)) {
        ESP_LOGW(TAG, "recording buffer off: %u KB of PSRAM free, it needs %u KB and %u KB to spare",
                 static_cast<unsigned>(free_bytes / 1024), static_cast<unsigned>(n / 1024),
                 static_cast<unsigned>(hg::RecordingBuffer::kPsramHeadroom / 1024));
        return nullptr;
      }
      void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      ESP_LOGI(TAG, "recording buffer %s: %u KB of PSRAM, %u KB free before", p ? "on" : "off (allocation failed)",
               static_cast<unsigned>(n / 1024), static_cast<unsigned>(free_bytes / 1024));
      return p;
    };
#else
  profile.faces = false;
#endif
  profile.default_server_url = CONFIG_HG_DEFAULT_SERVER_URL;
  profile.default_access_token = CONFIG_HG_DEFAULT_ACCESS_TOKEN;
  profile.has_cancel_button = board.buttons.cancel >= 0 || touch;
  profile.has_scroll_buttons = (board.buttons.up >= 0 && board.buttons.down >= 0) || board.encoder.a >= 0;
  profile.wake_key = board.buttons.talk >= 0 || board.axp_power_key || board.pwr_key.enabled;
  profile.talk_key = board.buttons.talk >= 0;
  profile.talk_label = board.talk_label;
  profile.cancel_label = board.cancel_label;
#if CONFIG_HG_BOARD_CARDPUTER_ADV
  profile.has_cancel_button = keyboard;
  profile.has_scroll_buttons = keyboard;
#endif
  if (touch && board.touch.enabled) {
    profile.touch_screen = true;
    profile.touch_ppi = board.touch.ppi;
    profile.extra_settings = {"touch_cancel"};
  }
  profile.extra_settings.push_back("timezone");  // the port keeps its own time
  profile.motion_sensor = imu;
#if !CONFIG_HG_BOARD_ESP32_CYD
  if (hal.mic == &g_codec_mic) {
    profile.mic_rate = hgp::CodecAudio::kRate;
    profile.vad_min_rms = board.codec.vad_min_rms;
    profile.drop_silent_recordings = board.codec.drop_silent_recordings;
  }
  if (hal.speaker == &g_codec_speaker) profile.speaker_rate = hgp::CodecAudio::kRate;
  profile.tailnet = hgp::TailnetLink::supported();
  profile.sd_log = board.sd.enabled;
#endif

#if CONFIG_HG_BOARD_ESP32_CYD
  // From the heap, so the early framebuffer can take the one large contiguous
  // block before this claims part of what is left of internal RAM.
  static hg::App& app = *new hg::App(hal, profile);
#else
  static hg::App app(hal, profile);
#endif
  static hg::TouchGestures gestures(app);
  if (profile.touch_screen) g_gestures = &gestures;
  apply_touch_cancel();
  add_status_led(app, board.status_led);
#if CONFIG_HG_BOARD_WS_P4_TOUCH_LCD_7B || CONFIG_HG_BOARD_CORES3
  add_camera_action(app, i2c_bus);
#endif
  app.on_setting_changed = [](std::string_view key) {
    if (key == "wifi_ssid" || key == "wifi_pass") app.close_wifi_setup();
    if (key == "wifi_ssid" || key == "wifi_pass" || key == "wifi_nets") g_wifi.networks_changed();
    if (key == "touch_cancel") apply_touch_cancel();
    if (key == "clock") g_time.set_wanted(app.clock_enabled());
    if (key == "timezone") g_time.apply_timezone(g_storage);
#if !CONFIG_HG_BOARD_ESP32_CYD
    if (key == "touch_wake") g_touch.set_wakes(app.touch_wake());
    if (key == "sd_log") g_sdlog.set_enabled(app.sd_log());
    if (key == "name" && hgp::board_config().bluetooth) g_phone.set_name(device_name());
    if (key == "tailnet" || key == "ts_key") g_tailnet.settings_changed();
#endif
  };
#if !CONFIG_HG_BOARD_ESP32_CYD
  if (board.bluetooth && g_phone.begin(device_name())) {
    app.on_phone_pair = [] { return g_phone.open_pairing(); };
    app.on_phone_pair_close = [] { g_phone.close_pairing(); };
    app.on_phone_forget = [] { g_phone.forget(); };
  }
  if (profile.tailnet) {
    g_tailnet.begin(app, device_name());
    app.on_tailnet_forget = [] { return g_tailnet.forget(); };
  }
#endif
  if (board.radio_enabled) {
    app.on_wifi_setup = [] { return g_wifi.start_setup(); };
    app.on_wifi_setup_close = [] { return g_wifi.stop_setup(); };
    app.on_wifi_setup_ap = [] { return g_wifi.setup_ap(); };
    app.current_wifi = [] { return g_wifi.current(); };
  }
  app.on_diag = [](hg::json::Value& report) {
    hgp::diag::report(report);
    report.set("ota", g_updater.describe());
#if !CONFIG_HG_BOARD_ESP32_CYD
    if (const auto ma = g_power.charge_current_ma()) report.set("charge_current_ma", static_cast<unsigned>(*ma));
    if (hgp::board_config().tailnet) g_tailnet.report(report);
#endif
  };
  app.recent_log = &hgp::diag::recent_log;
  app.on_log_level = [](const std::string& tag, const std::string& level) {
    static const esp_log_level_t kLevels[] = {ESP_LOG_NONE, ESP_LOG_ERROR, ESP_LOG_WARN, ESP_LOG_INFO, ESP_LOG_DEBUG};
    static_assert(std::size(kLevels) == std::size(hg::App::kLogLevels));
    for (size_t i = 0; i < std::size(kLevels); ++i) {
      if (level != hg::App::kLogLevels[i]) continue;
      esp_log_level_set(tag.c_str(), kLevels[i]);
      ESP_LOGI(TAG, "log level of %s: %s", tag.c_str(), level.c_str());
      return true;
    }
    return false;
  };
  if (board.light_sleep) keep_pins_through_sleep(board);
#if !CONFIG_HG_BOARD_ESP32_CYD
  g_system.on_doze_start = [] { g_codec_speaker.power_down(); };
  hgp::EspSystem::on_sleep_held_change = [] { g_touch.nudge(); };
  g_system.on_doze_end = [] {
    g_touch.nudge();
    g_codec_speaker.power_up();
    g_wifi.retry_now();
  };
#endif
  app.begin();
  g_sdlog.set_enabled(app.sd_log());  // the stored setting wins over the default
#if !CONFIG_HG_BOARD_ESP32_CYD
  g_touch.set_wakes(app.touch_wake());
#endif
  g_time.set_wanted(app.clock_enabled());  // SNTP only while the clock face is on
  hgp::console::begin();

#if !CONFIG_HG_BOARD_ESP32_CYD
  TickType_t motion_at = 0;     // last motion poll (the loop runs at ~10 ms)
  uint32_t motion_samples = 0;  // the motion task's count at the last forwarded sample
#endif

  for (;;) {
    hgp::Event ev;
    // Block briefly for events, then run the core's timers and animations.
    // Dozing on a light-sleep board, the chip may sleep between events; input and
    // the network still wake it. Other boards keep the short wait.
    if (hgp::events::receive(ev, pdMS_TO_TICKS(board.light_sleep && app.dozing() ? hg::App::kDozeWaitMs : 10))) {
      do {
        dispatch(app, ev);
        hgp::events::release(ev);
      } while (hgp::events::receive(ev, 0));
    }
    // A loss whose WsClosed event was dropped; harmless if it was delivered too.
    if (g_transport.take_lost()) app.on_transport_closed("disconnected");
    g_transport.set_metering(app.in_turn());  // the uplink line, while recording, waiting or replying
    // Waiting would leave the device answering nothing until someone resets it.
    if (g_transport.stuck()) {
      ESP_LOGE(TAG, "the WebSocket send task is stuck; restarting");
      vTaskDelay(pdMS_TO_TICKS(100));  // let the log line out
      esp_restart();
    }
    g_buttons.poll(app);
#if CONFIG_HG_BOARD_CARDPUTER_ADV
    if (hal.display == &g_compose) g_keyboard.poll(app, g_compose);
#endif
    g_wifi.tick(app, g_system.now_ms());
#if !CONFIG_HG_BOARD_ESP32_CYD
    g_tailnet.tick(g_system.now_ms());
#endif
    if (g_gestures) g_gestures->tick(g_system.now_ms());
    app.tick();
#if CONFIG_HG_BOARD_CORES3
    // The camera pipeline's lifecycle: start when the camera mode opens, stop
    // when it leaves, and stop 60 s after the last capture (the mode, when a
    // capture ran on its own). A start that fails closes the mode and hides
    // the Camera item for the rest of the boot.
    static bool camera_mode_was = false;
    const bool camera_mode_now = app.camera_open();
    if (camera_mode_now != camera_mode_was) {
      camera_mode_was = camera_mode_now;
      if (camera_mode_now) {
        ensure_camera_run();  // a failed start closes the mode below on the next pass
        if (!hgp::camera_hal_running()) app.close_camera();  // the item hides; the menu comes back
      } else {
        stop_camera();
      }
    } else if (!camera_mode_now && hgp::camera_hal_running() &&
               xTaskGetTickCount() - camera_use_ms >= pdMS_TO_TICKS(60000)) {
      stop_camera();
    }
#endif
#if !CONFIG_HG_BOARD_ESP32_CYD
    // The motion task keeps the latest sample; poll it about every 100 ms instead
    // of letting 10 Hz events fill the queue. Only new reads go to the app, so a
    // sensor that stops answering still goes stale there.
    if (imu && xTaskGetTickCount() - motion_at >= pdMS_TO_TICKS(100)) {
      motion_at = xTaskGetTickCount();
      const uint32_t n = g_motion.samples();
      if (n != motion_samples) {
        motion_samples = n;
        int16_t mx, my, mz;
        g_motion.latest(mx, my, mz);
        app.on_motion(mx, my, mz);
      }
    }
#endif
  }
#endif  // startup mode: diagnostic modes run and halt above
}
