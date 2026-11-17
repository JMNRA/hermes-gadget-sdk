// Serial console: every line goes to hg::App::console() on the app task, so the
// same commands work here, in the simulator and from `hermes-gadget provision`.
#include "port.hpp"  // first: pulls in FreeRTOS.h ahead of task.h/queue.h

#include <cstdio>
#include <cstring>
#include <string>

#include "board.hpp"
#include "esp_console.h"
#include "esp_log.h"
#include "hg/console_args.hpp"
#include "sdkconfig.h"
#if CONFIG_HG_BOARD_M5STACK_TAB5
#include "driver/gpio.h"
#include "esp_timer.h"
#include "panel_tab5.hpp"
#endif
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"
#endif

namespace hgp::console {
namespace {

const char* TAG = "hg.console";

#if CONFIG_HG_BOARD_M5STACK_TAB5
// The Tab5 PWR probe (panel_tab5.hpp): the whole hardware surface it is allowed
// to touch, in one place, and all of it read-only. It answers here on the
// console task rather than in the app so it works even while that task is stuck,
// and it never writes an expander register: 0x43 and 0x44 are only read, so the
// charger, the amplifier and every enabled output stay exactly as they were.
struct Tab5PwrProbeIo {
  i2c_master_dev_handle_t expander[2] = {nullptr, nullptr};

  i2c_master_dev_handle_t device(uint8_t address) const {
    if (address == kTab5PwrProbeExpanderA) return expander[0];
    if (address == kTab5PwrProbeExpanderB) return expander[1];
    return nullptr;
  }
  bool read(uint8_t address, uint8_t reg, uint8_t& value, uint32_t timeout_ms) {
    i2c_master_dev_handle_t dev = device(address);
    if (!dev) return false;
    return i2c_master_transmit_receive(dev, &reg, 1, &value, 1, static_cast<int>(timeout_ms)) == ESP_OK;
  }
  uint64_t now_ms() const { return static_cast<uint64_t>(esp_timer_get_time() / 1000); }
  void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms) + 1); }
  void print(const char* line) { std::printf("%s\n", line); }
  // The pad's current state: the pull to restore, and the evidence that nobody
  // has claimed it yet.
  bool pin_state(int gpio, Tab5PwrProbePinConfig& state) {
    gpio_io_config_t io = {};
    if (gpio_get_io_config(static_cast<gpio_num_t>(gpio), &io) != ESP_OK) return false;
    state.output = io.oe != 0;
    state.pull_up = io.pu != 0;
    state.pull_down = io.pd != 0;
    state.input = io.ie != 0;
    state.od = io.od != 0;
    state.oe_inv = io.oe_inv != 0;
    state.fun_sel = static_cast<uint8_t>(io.fun_sel);
    return true;
  }
  // The only change this probe ever makes to a pad: its internal pull-up. No
  // level, no direction, no IO_MUX function.
  bool pin_pull_up(int gpio) {
    return gpio_set_pull_mode(static_cast<gpio_num_t>(gpio), GPIO_PULLUP_ONLY) == ESP_OK;
  }
  void pin_restore(int gpio, const Tab5PwrProbePinConfig& state) {
    const gpio_pull_mode_t pull = state.pull_up && state.pull_down ? GPIO_PULLUP_PULLDOWN
                                   : state.pull_up            ? GPIO_PULLUP_ONLY
                                   : state.pull_down          ? GPIO_PULLDOWN_ONLY
                                                              : GPIO_FLOATING;
    if (gpio_set_pull_mode(static_cast<gpio_num_t>(gpio), pull) != ESP_OK)
      ESP_LOGW(TAG, "pwrprobe: pad %d kept a pull-up", gpio);
  }
  int pin_level(int gpio) { return gpio_get_level(static_cast<gpio_num_t>(gpio)); }
};

int run_tab5_pwr_probe(int argc, char** argv) {
  const BoardConfig& board = board_config();
  i2c_master_bus_handle_t bus = i2c::bus(board.i2c);
  if (!bus) {
    std::printf("@error pwrprobe: no I2C bus on this board\n");
    return 1;
  }
  Tab5PwrProbeIo io;
  const uint8_t addresses[2] = {kTab5PwrProbeExpanderA, kTab5PwrProbeExpanderB};
  for (size_t i = 0; i < 2; ++i) {
    i2c_device_config_t config = {};
    config.device_address = addresses[i];
    config.scl_speed_hz = board.i2c.hz;
    // A handle of our own on the shared bus: the app's drivers keep theirs, and
    // only reads follow.
    if (i2c_master_bus_add_device(bus, &config, &io.expander[i]) != ESP_OK) io.expander[i] = nullptr;
  }
  std::string args;
  for (int i = 1; i < argc; ++i) {
    if (i > 1) args.push_back(' ');  // extra words reach the parser so they are refused, not dropped
    args += argv[i];
  }
  Tab5PwrProbe<Tab5PwrProbeIo> probe(io);
  const bool ran = probe.run(args);
  for (i2c_master_dev_handle_t& dev : io.expander)
    if (dev) i2c_master_bus_rm_device(dev);
  return ran ? 0 : 1;
}
#endif

// The Tab5 battery probe (panel_tab5.hpp): the INA226 the BSP README names, if it
// is really there. Its only device handles are the two candidate addresses, and
// its only writes are the chip's own calibration and config registers -- the
// charger enable pins and the amplifier live on the two expander chips, which
// this adapter cannot address at all.
struct Tab5BattIo {
  i2c_master_dev_handle_t chip[2] = {nullptr, nullptr};

  i2c_master_dev_handle_t device(uint8_t address) const {
    if (address == kTab5BattAddressA) return chip[0];
    if (address == kTab5BattAddressB) return chip[1];
    return nullptr;
  }
  // Big endian on the wire, as the INA226's register map is.
  bool read16(uint8_t address, uint8_t reg, uint16_t& value, uint32_t timeout_ms) {
    i2c_master_dev_handle_t dev = device(address);
    if (!dev) return false;
    uint8_t bytes[2] = {0, 0};
    if (i2c_master_transmit_receive(dev, &reg, 1, bytes, 2, static_cast<int>(timeout_ms)) != ESP_OK) return false;
    value = static_cast<uint16_t>((bytes[0] << 8) | bytes[1]);
    return true;
  }
  bool write16(uint8_t address, uint8_t reg, uint16_t value, uint32_t timeout_ms) {
    i2c_master_dev_handle_t dev = device(address);
    if (!dev) return false;
    const uint8_t bytes[3] = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xff)};
    return i2c_master_transmit(dev, bytes, sizeof(bytes), static_cast<int>(timeout_ms)) == ESP_OK;
  }
  void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms) + 1); }
  void print(const char* line) { std::printf("%s\n", line); }
};

int run_tab5_batt(int argc, char** argv) {
  const BoardConfig& board = board_config();
  i2c_master_bus_handle_t bus = i2c::bus(board.i2c);
  if (!bus) {
    std::printf("@error batt: no I2C bus on this board\n");
    return 1;
  }
  Tab5BattIo io;
  const uint8_t addresses[2] = {kTab5BattAddressA, kTab5BattAddressB};
  for (size_t i = 0; i < 2; ++i) {
    i2c_device_config_t config = {};
    config.device_address = addresses[i];
    config.scl_speed_hz = board.i2c.hz;
    // A handle of our own on the shared bus: the codec keeps its own, and only
    // reads go to an address that is not an INA226.
    if (i2c_master_bus_add_device(bus, &config, &io.chip[i]) != ESP_OK) io.chip[i] = nullptr;
  }
  std::string args;
  for (int i = 1; i < argc; ++i) {
    if (i > 1) args.push_back(' ');  // extra words reach the parser so they are refused, not dropped
    args += argv[i];
  }
  Tab5BatteryProbe<Tab5BattIo> probe(io);
  const bool ran = probe.run(args);
  for (i2c_master_dev_handle_t& dev : io.chip)
    if (dev) i2c_master_bus_rm_device(dev);
  return ran ? 0 : 1;
}

int run(int argc, char** argv) {
#if CONFIG_HG_BOARD_M5STACK_TAB5
  // This board only: a bounded, read-only register watch for the missing PWR
  // input. It needs no app task, so it answers here before the queue below.
  if (argc && std::strcmp(argv[0], "pwrprobe") == 0) return run_tab5_pwr_probe(argc, argv);
  // And the battery question: is the INA226 the BSP README names on this bus?
  // One shot, and the only writes are that chip's own config registers.
  if (argc && std::strcmp(argv[0], "batt") == 0) return run_tab5_batt(argc, argv);
#endif
  const bool is_diag = argc && std::strcmp(argv[0], "diag") == 0;
  // The log copy needs no app task: it answers even while that task is stuck.
  if (is_diag && argc == 2 && std::strcmp(argv[1], "log") == 0) {
    std::string out = diag::recent_log();
    if (!out.empty() && out.back() != '\n') out += '\n';
    std::printf("%s@log end\n", out.c_str());
    return 0;
  }
  // ESP-IDF has already split the line and removed its quotes. `wifi` takes several
  // values that may hold spaces, so its arguments are quoted again.
  const bool quote = argc && std::strcmp(argv[0], "wifi") == 0;
  std::string line;
  for (int i = 0; i < argc; ++i) {
    if (i) line.push_back(' ');
    line += quote && i ? hg::quote_arg(argv[i]) : std::string(argv[i]);
  }
  // The app task answers on its own time. The slot outlives this function
  // if that answer comes after the wait below gave up.
  ConsoleRequest* req = ConsoleRequest::create();
  const bool posted = events::post(EventType::Console, line.data(), line.size(), 0, req);
  if (!posted) req->release();  // nobody will answer: drop the app task's share too
  if (posted && req->wait(3000)) {
    std::printf("%s\n", req->reply().c_str());
#if CONFIG_HG_BOARD_M5STACK_TAB5
    // `help` lists the app's commands; pwrprobe and batt answer in the port, so
    // their lines are added here and only on this board.
    if (argc == 1 && std::strcmp(argv[0], "help") == 0) {
      std::printf("%s\n", kTab5PwrProbeCommandHelp);
      std::printf("%s\n", kTab5BattCommandHelp);
    }
#endif
  } else if (is_diag && argc == 1) {
    // A stuck app task or a full event queue is what a hang report most needs to
    // show: the port answers for itself.
    std::printf("%s\n", diag::report_without_app().c_str());
  } else {
    std::printf("@error device busy\n");
  }
  req->release();
  return 0;
}

}  // namespace

void begin() {
  static const char* const kCommands[] = {"help",      "status", "diag",   "get",         "set",       "say",       "camera",
                                          "talk",      "release", "cancel", "new-session", "reconnect", "forget-key",
                                          "factory-reset", "settings", "wifi-setup", "wifi", "phone", "perf", "log", "face", "choose"};
  for (const char* name : kCommands) {
    esp_console_cmd_t cmd = {};
    cmd.command = name;
    cmd.help = "Hermes Gadget command (see: help)";
    cmd.func = &run;
    esp_console_cmd_register(&cmd);
  }
#if CONFIG_HG_BOARD_M5STACK_TAB5
  // Registered on the Tab5 build only: the probe reads this board's expanders and
  // pads, and no other board has that hardware.
  esp_console_cmd_t probe = {};
  probe.command = "pwrprobe";
  probe.help = "Tab5 only: watch both expanders' input/interrupt registers and a few free pads, 1..30 s, read-only";
  probe.func = &run;
  esp_console_cmd_register(&probe);
  esp_console_cmd_t batt = {};
  batt.command = "batt";
  batt.help = "Tab5 only: probe 0x40/0x41 for the INA226 and read one triggered conversion (bus/shunt/current/power)";
  batt.func = &run;
  esp_console_cmd_register(&batt);
#endif
  esp_console_repl_t* repl = nullptr;
  esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
  repl_cfg.prompt = "gadget>";
  repl_cfg.max_cmdline_length = 256;
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  // The USB Serial/JTAG block keeps its interrupt enables and status through a software reset,
  // and the driver installed below adds its two (bytes received, send buffer empty) without
  // clearing the rest; its handler acknowledges only those two. An M5Stack StopWatch came up
  // with BUS_RESET enabled as well (int_ena 0x0204) after esptool had talked to it, and without
  // it after a power cycle (0x0004). Plugged into USB while it ran, it went silent within
  // seconds and then restarted every 3.8 s from the interrupt watchdog inside
  // usb_serial_jtag_driver_install, until a reset over USB. So only the driver's own
  // interrupts stay enabled, and whatever else is pending is acknowledged.
  constexpr uint32_t kDriverInterrupts = USB_SERIAL_JTAG_INTR_SERIAL_OUT_RECV_PKT | USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY;
  const uint32_t enabled = usb_serial_jtag_ll_get_intr_ena_status();
  if (enabled & ~kDriverInterrupts)
    ESP_LOGW(TAG, "usb serial/jtag had interrupts enabled that nothing handles (int_ena 0x%04x); disabling them",
             static_cast<unsigned>(enabled));
  usb_serial_jtag_ll_disable_intr_mask(~kDriverInterrupts);
  usb_serial_jtag_ll_clr_intsts_mask(~kDriverInterrupts);
  esp_console_dev_usb_serial_jtag_config_t dev = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&dev, &repl_cfg, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
  esp_console_dev_usb_cdc_config_t dev = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&dev, &repl_cfg, &repl));
#else
  esp_console_dev_uart_config_t dev = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_console_new_repl_uart(&dev, &repl_cfg, &repl));
#endif
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  // With no USB host, ESP-IDF's read fails at once, and its console task then asks again every
  // 10 ms. A task that runs every tick leaves the idle task less than the three ticks tickless
  // idle wants, so a board started on battery never took light sleep while dozing: an M5Stack
  // StopWatch logged 0 s of it in 45 minutes of doze, and its usual share from the first plug-in
  // on, when the read blocks and stays blocked after the cable is gone. So the console starts
  // once a host is there. The wait looks twice a second, which light sleep fits between.
  const BaseType_t waiting = xTaskCreate(
      [](void* arg) {
        do vTaskDelay(pdMS_TO_TICKS(500));
        while (!usb_serial_jtag_is_connected());
        if (esp_console_start_repl(static_cast<esp_console_repl_t*>(arg)) == ESP_OK)
          ESP_LOGI(TAG, "console ready (type: help)");
        vTaskDelete(nullptr);
      },
      "hg-console-wait", 3072, repl, 1, nullptr);
  if (waiting != pdPASS) {
    // No memory for the wait: start as before, and say so. Without a host the console then asks
    // every 10 ms, which is the lesser loss beside having no console.
    ESP_LOGW(TAG, "no task to wait for a USB host; the console starts now");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    ESP_LOGI(TAG, "console ready (type: help)");
  }
#else
  ESP_ERROR_CHECK(esp_console_start_repl(repl));
  ESP_LOGI(TAG, "console ready (type: help)");
#endif
}

}  // namespace hgp::console
