#pragma once

#include <cstddef>
#include <cstdint>

#include "hg/hal.hpp"
#include "hg/crypto.hpp"
#include <algorithm>
#include <array>
#include <cstring>

namespace hgp {
// Never forwards a connection or message: this is not a production transport.
class Tab5OfflineTransport final : public hg::Transport {
 public:
  void connect(const std::string&, const std::string&) override { violated_ = true; }
  bool send_text(std::string_view) override { violated_ = true; return false; }
  bool send_binary(const uint8_t*, size_t) override { violated_ = true; return false; }
  void close() override {}
  bool violated() const { return violated_; }
 private:
  bool violated_ = false;
};

// One-time, RAM-only identity input for the explicitly offline diagnostic.
class Tab5OfflineSeed {
 public:
  ~Tab5OfflineSeed() { clear(); }
  Tab5OfflineSeed(const Tab5OfflineSeed&) = delete;
  Tab5OfflineSeed& operator=(const Tab5OfflineSeed&) = delete;
  Tab5OfflineSeed() = default;

  bool feed(const uint8_t* bytes, size_t len) {
    static constexpr uint8_t header[] = {'H','G','S','E','E','D','0','1'};
    if (failed_ || used_ || (!bytes && len) || len > sizeof(frame_) - size_) return fail();
    for (size_t i = 0; i < len; ++i) {
      if (size_ < sizeof(header) && bytes[i] != header[size_]) return fail();
      frame_[size_++] = bytes[i];
    }
    return true;
  }

  bool take(uint8_t* out, size_t len) {
    if (failed_ || used_ || size_ != sizeof(frame_) || !out || len != 32) return false;
    for (size_t i = 0; i < len; ++i) out[i] = frame_[i + 8];
    clear();
    used_ = true;
    return true;
  }

 private:
  void clear() {
    volatile uint8_t* p = frame_;
    for (size_t i = 0; i < sizeof(frame_); ++i) p[i] = 0;
    size_ = 0;
  }
  bool fail() { clear(); failed_ = true; return false; }
  uint8_t frame_[40] = {};
  size_t size_ = 0;
  bool failed_ = false;
  bool used_ = false;
};
// USB integrity handshake, NOT authentication or device-side freshness proof.
// All frames are 56 bytes: 8-byte tag, 16-byte host session ID, 32-byte
// seed/proof. Proof = SHA256("HG-TAB5-OFFLINE-V2" || ID || seed || challenge).
// HGREADY3 carries a 16-byte device challenge before the offer. A different
// challenge rejects an old ACK even if its complete offer survives USB drops.
// Challenge uniqueness/entropy is NOT established: repeated challenges permit
// complete replay after reset. An active peer can compute ACKs: not authentication.
// Missing/inserted bytes fail closed unless the surviving transcript is valid;
// this protocol cannot detect every USB drop, nor prove hardware execution.
inline void tab5_wipe(uint8_t* p, size_t n) {
  volatile uint8_t* v = p;
  while (n--) *v++ = 0;
}
class Tab5OfflineExchange {
 public:
  explicit Tab5OfflineExchange(uint64_t start, std::array<uint8_t,16> challenge = {})
      : start_(start), challenge_(challenge) {}
  ~Tab5OfflineExchange() { clear(); }
  Tab5OfflineExchange(const Tab5OfflineExchange&) = delete;
  Tab5OfflineExchange& operator=(const Tab5OfflineExchange&) = delete;
  bool feed(const uint8_t* p, size_t n, uint64_t now) {
    if (!live(now) || (state_ != Offer && state_ != Ack) || (!p && n) || n > 56-size_)
      return fail();
    for (size_t i=0; i<n; ++i) frame_[size_++] = p[i];
    if (size_ != 56) return true;
    if (state_ == Offer) {
      if (std::memcmp(frame_, "HGSEED02", 8)) return fail();
      std::memcpy(seed_, frame_+24, 32);
      std::memcpy(proof_, "HGPROOF2", 8);
      std::memcpy(proof_+8, frame_+8, 16);
      uint8_t input[82]{};
      std::memcpy(input, "HG-TAB5-OFFLINE-V2", 18);
      std::memcpy(input+18, frame_+8, 48);
      std::memcpy(input+66, challenge_.data(), challenge_.size());
      auto digest = hg::crypto::sha256(input, sizeof(input));
      std::memcpy(proof_+24, digest.data(), 32);
      tab5_wipe(input, sizeof(input));
      tab5_wipe(digest.data(), digest.size());
      state_ = Proof;
    } else {
      uint8_t difference = 0;
      for (size_t i=0; i<56; ++i)
        difference |= frame_[i] ^ (i<8 ? static_cast<uint8_t>("HGACK002"[i]) : proof_[i]);
      if (difference) return fail();
      state_ = Accepted;
    }
    tab5_wipe(frame_, sizeof(frame_)); size_ = 0;
    return true;
  }
  bool response(uint8_t* out, size_t n, uint64_t now) {
    if (!live(now) || state_ != Proof || !out || n != 56) return fail();
    std::memcpy(out, proof_, 56); state_ = Ack; return true;
  }
  bool awaiting_proof() const { return state_ == Proof; }
  bool accepted() const { return state_ == Accepted; }
  bool finish(Tab5OfflineSeed& seed, uint64_t now) {
    if (!live(now) || state_ != Accepted) return fail();
    uint8_t legacy[40]{};
    std::memcpy(legacy, "HGSEED01", 8);
    std::memcpy(legacy+8, seed_, 32);
    bool ok = seed.feed(legacy, sizeof(legacy));
    tab5_wipe(legacy, sizeof(legacy));
    clear(); state_ = Done; return ok;
  }
  bool cancel() { return fail(); }
  bool cleared() const {
    for (auto b : frame_) if (b) return false;
    for (auto b : seed_) if (b) return false;
    for (auto b : proof_) if (b) return false;
    return size_ == 0;
  }
 private:
  enum State { Offer, Proof, Ack, Accepted, Done, Failed };
  bool live(uint64_t now) const { return now >= start_ && now-start_ < 60000; }
  void clear() {
    tab5_wipe(frame_, sizeof(frame_)); tab5_wipe(seed_, sizeof(seed_));
    tab5_wipe(proof_, sizeof(proof_)); size_ = 0;
  }
  bool fail() { clear(); state_ = Failed; return false; }
  uint64_t start_;
  // Public, non-secret diagnostic discriminator; never used as identity entropy.
  std::array<uint8_t,16> challenge_;
  State state_ = Offer;
  size_t size_ = 0;
  uint8_t frame_[56]{}, seed_[32]{}, proof_[56]{};
};
}  // namespace hgp
