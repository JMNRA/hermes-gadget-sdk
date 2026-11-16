#pragma once

#include <cstdlib>
#include <map>
#include "hg/app.hpp"
#include "hg/touch.hpp"
#include "tab5_offline_seed.hpp"

namespace hgp {
// A bench board may boot with no host attached, so the offline build renders the
// confirmed preview first and only hands over to the real App on a verified seed.
// The host exchange is bounded: after this window the device stops waiting and
// keeps the preview. This is not a session timeout the App can extend.
inline constexpr uint64_t kTab5OfflineSeedWindowMs = 15000;

// Left: the offline preview keeps running. Right: the real hg::App.
enum class Tab5OfflineRoute { Preview, App };

inline bool tab5_offline_window_open(uint64_t start_ms, uint64_t now_ms) {
  return now_ms >= start_ms && now_ms - start_ms < kTab5OfflineSeedWindowMs;
}

// Only a completed handover inside the window reaches the App: `seed_ready` means
// the verified exchange finished and the complete seed is in RAM. Every other
// outcome (no host, timeout, rejected frame, late handover) keeps the preview
// running with touch feedback: no App, identity, NVS or network.
inline Tab5OfflineRoute tab5_offline_route(bool seed_ready, bool window_open) {
  return seed_ready && window_open ? Tab5OfflineRoute::App : Tab5OfflineRoute::Preview;
}

// Deliberately not EspSystem: there is no hardware RNG or reseed fallback.
class Tab5OfflineSystem final : public hg::System {
 public:
  using Clock = uint32_t (*)();
  using Stop = void (*)(); // must not return; abort defensively if it does
  Tab5OfflineSystem(Tab5OfflineSeed& seed, Clock clock, Stop stop)
      : seed_(seed), clock_(clock), stop_(stop) {}
  uint32_t now_ms() override { return clock_(); }
  void random_bytes(uint8_t* out, size_t len) override {
    if (consumed_ || !seed_.take(out, len)) { stop_(); std::abort(); }
    consumed_ = true;
  }
  // App messages may contain identity or settings. Never forward them to USB.
  void log(hg::LogLevel, std::string_view) override {}
  bool consumed() const { return consumed_; }
 private:
  Tab5OfflineSeed& seed_;
  Clock clock_;
  Stop stop_;
  bool consumed_ = false;
};

// Settings exist only for this diagnostic session. Identity is not retained
// here: App keeps its generated key in memory and handles a failed save.
class Tab5OfflineStorage final : public hg::Storage {
 public:
  std::optional<std::string> get(std::string_view key) override {
    auto it = values_.find(std::string(key));
    return it == values_.end() ? std::nullopt : std::optional<std::string>(it->second);
  }
  bool set(std::string_view key, std::string_view value) override {
    if (key == "device_key") return false;
    values_[std::string(key)] = std::string(value);
    return true;
  }
  void erase(std::string_view key) override { values_.erase(std::string(key)); }
 private:
  std::map<std::string, std::string> values_;
};

inline hg::Hal tab5_offline_hal(Tab5OfflineSystem& system,
                               Tab5OfflineTransport& transport, hg::Display& display,
                               Tab5OfflineStorage& storage) {
  hg::Hal hal;
  hal.system = &system;
  hal.transport = &transport;
  hal.display = &display;
  hal.storage = &storage;
  return hal; // no persistent store or network/audio services
}
inline hg::DeviceProfile tab5_offline_profile(uint16_t ppi) {
  hg::DeviceProfile profile;
  profile.board = "Tab5 OFFLINE diagnostic";
  profile.default_name = "Tab5 OFFLINE";
  profile.touch_screen = true;
  profile.touch_ppi = ppi;
  profile.has_cancel_button = true;
  profile.faces = false;
  return profile; // empty URL/token, no callbacks or external authentication
}
} // namespace hgp
