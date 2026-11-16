#include "port.hpp"  // first: pulls in FreeRTOS.h ahead of task.h/queue.h

#include <atomic>
#include <string>
#include <ctime>
#include <cstdlib>

#if CONFIG_HG_BOARD_M5STACK_TAB5
#include "bootloader_random.h"
#include "mbedtls/private/ctr_drbg.h"
#include "tab5_entropy.hpp"
#endif

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"

namespace hgp {

#if CONFIG_HG_BOARD_M5STACK_TAB5
namespace {
mbedtls_ctr_drbg_context tab5_drbg;
bool tab5_rng_ready = false;
bool tab5_entropy_open = false;

int tab5_entropy_callback(void*, unsigned char* out, size_t len) {
  // A later automatic reseed must fail rather than reopen the ADC while in use.
  if (!tab5_entropy_open) return MBEDTLS_ERR_CTR_DRBG_ENTROPY_SOURCE_FAILED;
  esp_fill_random(out, len);
  return 0;
}
}

void tab5_rng_init() {
  if (tab5_rng_ready) abort();
  mbedtls_ctr_drbg_init(&tab5_drbg);
  const bool seeded = tab5_seed_entropy(
      [] { bootloader_random_enable(); tab5_entropy_open = true; },
      [] { return mbedtls_ctr_drbg_seed(&tab5_drbg, tab5_entropy_callback, nullptr, nullptr, 0) == 0; },
      [] { tab5_entropy_open = false; bootloader_random_disable(); });
  if (!seeded) {
    mbedtls_ctr_drbg_free(&tab5_drbg);
    ESP_LOGE("hg.rng", "Tab5 entropy initialization failed");
    abort();
  }
  tab5_rng_ready = true;
}
#endif

uint32_t EspSystem::now_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

uint64_t EspSystem::now_us() { return static_cast<uint64_t>(esp_timer_get_time()); }

bool local_wall_time(hg::WallTime& out) {
  const time_t t = time(nullptr);
  struct tm tm{};
  // A fresh boot without RTC and network still reads 1970; a real date is
  // 2025 or later.
  if (!localtime_r(&t, &tm) || tm.tm_year + 1900 < 2025) return false;
  out.year = static_cast<uint16_t>(tm.tm_year + 1900);
  out.month = static_cast<uint8_t>(tm.tm_mon + 1);
  out.day = static_cast<uint8_t>(tm.tm_mday);
  out.weekday = static_cast<uint8_t>(tm.tm_wday);
  out.hour = static_cast<uint8_t>(tm.tm_hour);
  out.minute = static_cast<uint8_t>(tm.tm_min);
  out.second = static_cast<uint8_t>(tm.tm_sec);
  return true;
}

void EspSystem::random_bytes(uint8_t* out, size_t len) {
#if CONFIG_HG_BOARD_M5STACK_TAB5
  if (!tab5_rng_ready) abort();
  while (len) {
    const size_t n = len > MBEDTLS_CTR_DRBG_MAX_REQUEST ? MBEDTLS_CTR_DRBG_MAX_REQUEST : len;
    if (mbedtls_ctr_drbg_random(&tab5_drbg, out, n) != 0) {
      ESP_LOGE("hg.rng", "Tab5 random generator failed or requires reseeding");
      abort();  // Never return a partial key or nonce to the caller.
    }
    out += n;
    len -= n;
  }
#else
  // Existing profiles retain their existing hardware-RNG behavior.
  esp_fill_random(out, len);
#endif
}

void EspSystem::log(hg::LogLevel level, std::string_view message) {
  static const char* TAG = "hg";
  std::string msg(message);
  switch (level) {
    case hg::LogLevel::Debug: ESP_LOGD(TAG, "%s", msg.c_str()); break;
    case hg::LogLevel::Info: ESP_LOGI(TAG, "%s", msg.c_str()); break;
    case hg::LogLevel::Warn: ESP_LOGW(TAG, "%s", msg.c_str()); break;
    case hg::LogLevel::Error: ESP_LOGE(TAG, "%s", msg.c_str()); break;
  }
}

namespace {
// Light sleep taken since the doze began, for the log line that ends it.
std::atomic<int64_t> g_slept_us{0};
std::atomic<uint32_t> g_sleeps{0};
std::atomic<bool> g_dozing{false};

#if CONFIG_PM_LIGHT_SLEEP_CALLBACKS
esp_err_t count_sleep(int64_t slept_us, void*) {
  g_slept_us.fetch_add(slept_us, std::memory_order_relaxed);
  g_sleeps.fetch_add(1, std::memory_order_relaxed);
  return ESP_OK;
}
#endif
}  // namespace

bool EspSystem::enable_doze() {
#if CONFIG_PM_ENABLE && CONFIG_FREERTOS_USE_TICKLESS_IDLE
  if (awake_) return true;
#if CONFIG_PM_LIGHT_SLEEP_CALLBACKS
  esp_pm_sleep_cbs_register_config_t cbs = {};
  cbs.exit_cb = &count_sleep;
  esp_pm_light_sleep_register_cbs(&cbs);
#endif
  if (esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "hg-awake", &awake_) != ESP_OK) {
    awake_ = nullptr;
    return false;
  }
  esp_pm_lock_acquire(awake_);
  return true;
#else
  return false;
#endif
}

bool EspSystem::dozing_now() { return g_dozing.load(std::memory_order_relaxed); }

namespace {
std::atomic<bool> g_sleep_held{false};
}

void EspSystem::set_sleep_held(bool held) {
  if (g_sleep_held.exchange(held) == held) return;
  if (on_sleep_held_change) on_sleep_held_change();
}

bool EspSystem::sleep_held_now() { return g_sleep_held.load(std::memory_order_relaxed); }

void EspSystem::set_dozing(bool dozing) {
  if (!awake_ || dozing == dozing_) return;
  dozing_ = dozing;
  if (dozing) {
    g_slept_us = 0;
    g_sleeps = 0;
    doze_started_us_ = esp_timer_get_time();
    if (on_doze_start) on_doze_start();
    g_dozing = true;
    ESP_LOGI("hg.power", "dozing: light sleep allowed");
    esp_pm_lock_release(awake_);
    return;
  }
  esp_pm_lock_acquire(awake_);
  g_dozing = false;
  if (on_doze_end) on_doze_end();
  const int64_t dozed_ms = (esp_timer_get_time() - doze_started_us_) / 1000;
  const int64_t slept_ms = g_slept_us.load() / 1000;
  ESP_LOGI("hg.power", "awake after %lld s dozing: light sleep %lld s (%d%%), %u wake-ups", dozed_ms / 1000,
           slept_ms / 1000, dozed_ms > 0 ? static_cast<int>(slept_ms * 100 / dozed_ms) : 0,
           static_cast<unsigned>(g_sleeps.load()));
}

bool EspSystem::wall_time(hg::WallTime& out) { return local_wall_time(out); }

}  // namespace hgp
