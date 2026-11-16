#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "axp2101.hpp"
#include "stc8_battery.hpp"
#include "cores3.hpp"
#include "sticks3.hpp"
#include "stopwatch.hpp"
#include "check.hpp"
#include "hg/battery.hpp"
#include "hg/ui.hpp"
#include "../esp32/main/panel_tab5.hpp"
#include "../esp32/main/tab5_entropy.hpp"
#include "../esp32/main/tab5_diagnostic.hpp"
#include "../esp32/main/tab5_offline_seed.hpp"
#include "../esp32/main/tab5_offline_app.hpp"
#include "../esp32/main/tab5_speaker_diagnostic.hpp"
#include "../esp32/main/tab5_mic_diagnostic.hpp"
#include "../esp32/main/tab5_audio_ui.hpp"

namespace {
struct SpeakerDiagnosticFake {
  // 0x100+reg read, 0x200+reg write, 1 open_dac, 2 set_mute, 3 tone, 4 close_dac
  std::array<uint8_t, 256> regs{};
  std::vector<int> calls;
  std::vector<uint8_t> writes;  // expander registers written, in order
  std::vector<bool> mutes;      // every set_mute() argument
  unsigned operations = 0, fail_at = 0, corrupt_after = 0;
  bool apply_failed_write = false, cleanup_fails = false, close_fails = false;
  uint64_t time = 0;
  uint32_t tone_delay = 100, open_delay = 0;
  uint32_t read_delay = 0, write_delay = 0, mute_delay = 0, close_delay = 0;
  bool fail_after_tone = false, corrupt_readback = false;
  unsigned tones = 0;
  uint8_t volume = 0;
  bool opened_muted = false;
  bool step(int call) {
    calls.push_back(call);
    return ++operations != fail_at && !cleanup_fails;
  }
  uint64_t now_ms() const { return time; }
  bool read(uint8_t reg, uint8_t& value, uint32_t timeout) {
    CHECK(timeout > 0 && timeout <= 50);
    time += read_delay;
    value = regs[reg];
    const bool verification = !calls.empty() && calls.back() == 0x200 + reg;
    if (verification && (corrupt_readback || (corrupt_after && writes.size() >= corrupt_after)))
      value ^= 0x02;
    return step(0x100 + reg);
  }
  bool write(uint8_t reg, uint8_t value, uint32_t timeout) {
    CHECK(timeout > 0 && timeout <= 50);
    time += write_delay;
    const bool ok = step(0x200 + reg);
    if (ok || apply_failed_write) {
      regs[reg] = value;
      writes.push_back(reg);
    }
    return ok;
  }
  bool open_dac(uint8_t volume, bool muted, uint32_t timeout) {
    CHECK(timeout > 0 && timeout <= 500);
    // DAC only, conservative volume, muted by default, and only once the
    // amplifier request was acknowledged and read back.
    CHECK(volume > 0 && volume <= 50);
    CHECK(muted);
    CHECK((regs[0x03] & 0x02) == 0x02);
    this->volume = volume;
    opened_muted = muted;
    time += open_delay;
    return step(1);
  }
  bool set_mute(bool muted, uint32_t timeout) {
    CHECK(timeout > 0 && timeout <= 50);
    mutes.push_back(muted);
    time += mute_delay;
    return step(2);
  }
  bool tone(const int16_t* pcm, size_t count, uint32_t timeout) {
    CHECK(timeout > 0 && timeout <= 1000);
    CHECK((regs[0x05] & 0x02) == 0x02);  // the amplifier request is on while writing
    CHECK(count == hgp::kTab5SpeakerToneFrames);
    for (size_t i = 0; i < count; ++i)
      CHECK(pcm[i] == (i % 24 < 12 ? hgp::kTab5SpeakerTonePeak : -hgp::kTab5SpeakerTonePeak));
    ++tones;
    time += tone_delay;
    const bool ok = step(3);
    if (fail_after_tone) cleanup_fails = true;
    return ok;
  }
  bool close_dac(uint32_t timeout) {
    CHECK(timeout > 0 && timeout <= 500);
    time += close_delay;
    return step(4) && !close_fails;
  }
};
}

TEST("Tab5 speaker diagnostic stays silent until explicit one-shot action") {
  SpeakerDiagnosticFake io;
  io.regs.fill(0xff);
  hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
  CHECK(io.calls.empty());
  CHECK(!diagnostic.play_once(false));
  CHECK(io.calls.empty());
  CHECK(diagnostic.prepare());
  CHECK(io.tones == 0);
  CHECK_EQ(io.volume, hgp::kTab5SpeakerVolume);
  CHECK(io.opened_muted);
  const auto ready_calls = io.calls;
  const auto ready_regs = io.regs;
  CHECK(!diagnostic.play_once(false));
  CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Ready);
  CHECK(io.calls == ready_calls && io.regs == ready_regs);
  // The reviewed speaker-off state: requested and read back before audio starts.
  CHECK((io.regs[0x05] & 0x02) == 0);
  CHECK((io.regs[0x07] & 0x02) == 0);
  CHECK((io.regs[0x03] & 0x02) == 0x02);
  CHECK(diagnostic.play_once(true));
  CHECK(io.tones == 1);
  CHECK((io.regs[0x03] & 0x02) == 0x02);  // dropped again after the single tone
  CHECK(diagnostic.state() == hgp::Tab5SpeakerState::SafeOff);
  CHECK_EQ(hgp::tab5_speaker_state_name(diagnostic.state()), std::string("SafeOff"));
  CHECK(!diagnostic.play_once(true));  // one attempt per boot, no repeat
  CHECK(!diagnostic.prepare());
  CHECK(!diagnostic.disarm());
  CHECK(io.tones == 1);
  CHECK(io.calls.back() == 4);                       // the DAC is closed last
  CHECK(io.volume == hgp::kTab5SpeakerVolume);
  CHECK(io.mutes == std::vector<bool>({false, true}));  // unmuted only for the tone
  CHECK(io.writes == std::vector<uint8_t>({0x05, 0x07, 0x03, 0x05, 0x05, 0x07, 0x03}));
  // Every expander write is a masked read-modify-write: read, write, read back,
  // and only registers 0x03/0x05/0x07 are ever touched.
  CHECK(io.calls.front() == 0x105);
  for (size_t i = 0; i < io.calls.size(); ++i) {
    const int call = io.calls[i];
    if (call < 0x100) continue;
    const uint8_t reg = static_cast<uint8_t>(call & 0xff);
    CHECK(reg == 0x03 || reg == 0x05 || reg == 0x07);
    if (call >= 0x200) {
      CHECK(i > 0 && io.calls[i - 1] == 0x100 + reg);
      CHECK(i + 1 < io.calls.size() && io.calls[i + 1] == 0x100 + reg);
    }
  }
  const auto opened = std::find(io.calls.begin(), io.calls.end(), 1);
  const auto played = std::find(io.calls.begin(), io.calls.end(), 3);
  CHECK(opened < played);
}

TEST("Tab5 speaker leaves the amplifier off when no deliberate start arrives") {
  SpeakerDiagnosticFake io;
  io.regs.fill(0xff);
  hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
  CHECK(diagnostic.prepare());
  CHECK(diagnostic.disarm());
  CHECK(io.tones == 0);
  CHECK(io.writes == std::vector<uint8_t>({0x05, 0x07, 0x03, 0x05, 0x07, 0x03}));  // never enables
  CHECK(io.mutes.empty());
  CHECK(io.calls.back() == 4);
  CHECK((io.regs[0x05] & 0x02) == 0);
  CHECK((io.regs[0x07] & 0x02) == 0);
  CHECK((io.regs[0x03] & 0x02) == 0x02);
  CHECK(diagnostic.state() == hgp::Tab5SpeakerState::SafeOff);
  CHECK(!diagnostic.disarm());  // one attempt per boot
  // The same state machine reports Unknown when the final readback disagrees.
  SpeakerDiagnosticFake bad;
  bad.regs.fill(0xff);
  bad.corrupt_after = 4;
  hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> refused(bad);
  CHECK(refused.prepare());
  CHECK(!refused.disarm());
  CHECK(refused.state() == hgp::Tab5SpeakerState::Unknown);
  CHECK_EQ(hgp::tab5_speaker_state_name(refused.state()), std::string("Unknown"));
}

TEST("Tab5 speaker probe: only a deliberate start byte plays the tone") {
  CHECK(hgp::tab5_speaker_start_byte('s') == hgp::Tab5SpeakerStart::Start);
  CHECK(hgp::tab5_speaker_start_byte('S') == hgp::Tab5SpeakerStart::Start);
  CHECK(hgp::tab5_speaker_start_byte('\r') == hgp::Tab5SpeakerStart::Ignore);
  CHECK(hgp::tab5_speaker_start_byte('\n') == hgp::Tab5SpeakerStart::Ignore);
  CHECK(hgp::tab5_speaker_start_byte(0) == hgp::Tab5SpeakerStart::Refuse);
  CHECK(hgp::tab5_speaker_start_byte('x') == hgp::Tab5SpeakerStart::Refuse);
  CHECK(hgp::tab5_speaker_start_byte(-1) == hgp::Tab5SpeakerStart::Refuse);
}

TEST("Tab5 speaker probe: the last line states SafeOff or Unknown, never both") {
  std::array<char, 64> line{};
  const size_t safe = hgp::tab5_speaker_result_line(line.data(), line.size(),
                                                    hgp::Tab5SpeakerState::SafeOff);
  CHECK_EQ(std::string(line.data(), safe), std::string("speaker probe: result SafeOff"));
  const size_t unknown = hgp::tab5_speaker_result_line(line.data(), line.size(),
                                                       hgp::Tab5SpeakerState::Unknown);
  CHECK_EQ(std::string(line.data(), unknown), std::string("speaker probe: result Unknown"));
  CHECK_EQ(hgp::tab5_speaker_result_line(line.data(), safe, hgp::Tab5SpeakerState::SafeOff), size_t{0});
  CHECK_EQ(hgp::tab5_speaker_result_line(line.data(), safe + 1, hgp::Tab5SpeakerState::SafeOff), safe);
  CHECK_EQ(hgp::tab5_speaker_result_line(nullptr, 64, hgp::Tab5SpeakerState::SafeOff), size_t{0});
  CHECK_EQ(hgp::tab5_speaker_result_line(line.data(), 0, hgp::Tab5SpeakerState::Unknown), size_t{0});
  CHECK_EQ(std::string(hgp::tab5_speaker_state_name(hgp::Tab5SpeakerState::Ready)), std::string("Ready"));
  CHECK_EQ(std::string(hgp::tab5_speaker_state_name(hgp::Tab5SpeakerState::Playing)), std::string("Playing"));
  CHECK(std::string(hgp::tab5_speaker_state_name(hgp::Tab5SpeakerState::SafeOff)) !=
        std::string(hgp::tab5_speaker_state_name(hgp::Tab5SpeakerState::Unknown)));
}

TEST("Tab5 speaker probe: one 1 kHz 250 ms tone at -40 dBFS") {
  static_assert(hgp::kTab5SpeakerToneFrames == 6000, "250 ms at 24 kHz");
  CHECK_EQ(hgp::kTab5SpeakerRate, 24000u);
  CHECK_EQ(hgp::kTab5SpeakerToneHz, 1000u);
  CHECK_EQ(hgp::kTab5SpeakerToneMs, 250u);
  CHECK(hgp::kTab5SpeakerVolume <= 50);
  CHECK(hgp::kTab5SpeakerVolume > 0);
  std::array<int16_t, hgp::kTab5SpeakerToneFrames> pcm{};
  CHECK_EQ(hgp::tab5_speaker_tone(pcm.data(), pcm.size()), pcm.size());
  int16_t peak = 0;
  for (size_t i = 0; i < pcm.size(); ++i) {
    // 24 frames are one period at 24 kHz: 12 high, then 12 low.
    CHECK(pcm[i] == (i % 24 < 12 ? hgp::kTab5SpeakerTonePeak : -hgp::kTab5SpeakerTonePeak));
    peak = std::max(peak, pcm[i]);
  }
  CHECK_EQ(peak, hgp::kTab5SpeakerTonePeak);
  // 32768 * 10^(-40/20) = 327.7: about -40 dBFS, never near full scale.
  const double dbfs = 20.0 * std::log10(double(peak) / 32768.0);
  CHECK(dbfs < -39.9 && dbfs > -40.2);
  // The generator never writes past the caller's capacity.
  std::array<int16_t, 4> small{};
  CHECK_EQ(hgp::tab5_speaker_tone(small.data(), small.size()), small.size());
  CHECK_EQ(small[0], hgp::kTab5SpeakerTonePeak);
  CHECK_EQ(small[1], hgp::kTab5SpeakerTonePeak);
  CHECK_EQ(hgp::tab5_speaker_tone(small.data(), 0), size_t{0});
  CHECK_EQ(hgp::tab5_speaker_tone(nullptr, 4), size_t{0});
}

TEST("Tab5 speaker probe: TX-only pin map and amplifier expander facts") {
  const auto pins = hgp::tab5_speaker_pins();
  CHECK_EQ(pins.mclk, 30);
  CHECK_EQ(pins.bclk, 27);
  CHECK_EQ(pins.ws, 29);
  CHECK_EQ(pins.dout, 26);
  CHECK_EQ(int(hgp::kTab5SpeakerExpanderAddr), 0x43);
  CHECK_EQ(int(hgp::kTab5SpeakerPaPin), 1);
  CHECK_EQ(int(hgp::kTab5SpeakerMask), 1 << hgp::kTab5SpeakerPaPin);
}

TEST("Tab5 speaker failures including applied writes remain unknown and never retry") {
  SpeakerDiagnosticFake baseline;
  hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> happy(baseline);
  CHECK(happy.prepare());
  CHECK(happy.play_once(true));
  CHECK(happy.state() == hgp::Tab5SpeakerState::SafeOff);
  for (unsigned fail = 1; fail <= baseline.operations; ++fail) {
    for (bool applies : {false, true}) {
      SpeakerDiagnosticFake io;
      io.regs.fill(0xff);
      io.fail_at = fail;
      io.apply_failed_write = applies;
      hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
      const bool prepared = diagnostic.prepare();
      const bool played = prepared && diagnostic.play_once(true);
      CHECK(!played);
      CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Unknown);
      CHECK(io.calls.back() == 4);  // the DAC close is still attempted
      CHECK(!diagnostic.play_once(true));
      CHECK(!diagnostic.prepare());
      CHECK(!diagnostic.disarm());
      CHECK(io.tones <= 1);
      CHECK((io.regs[0x05] & ~0x02u) == 0xfd);
      CHECK((io.regs[0x07] & ~0x02u) == 0xfd);
      CHECK((io.regs[0x03] & ~0x02u) == 0xfd);
    }
  }
}

TEST("Tab5 speaker rejects late instruction, tone and close callbacks") {
  for (int late = 0; late < 5; ++late) {
    SpeakerDiagnosticFake io;
    hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
    CHECK(diagnostic.prepare());
    if (late == 0) io.read_delay = 51;
    if (late == 1) io.write_delay = 51;  // the write applies despite the deadline miss
    if (late == 2) io.mute_delay = 51;
    if (late == 3) io.tone_delay = 1001;  // longer than the driver's own wait
    if (late == 4) io.close_delay = 501;
    CHECK(!diagnostic.play_once(true));
    CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Unknown);
    CHECK(io.tones == (late >= 3 ? 1u : 0u));
    const auto calls = io.calls;
    CHECK(!diagnostic.play_once(true));
    CHECK(!diagnostic.prepare());
    CHECK(!diagnostic.disarm());
    CHECK(io.calls == calls);  // no retry, ever
  }
}

TEST("Tab5 speaker rejects readback mismatch, late open and failed cleanup") {
  {
    SpeakerDiagnosticFake io;
    io.corrupt_readback = true;
    hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
    CHECK(!diagnostic.prepare());
    CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Unknown);
    CHECK(!diagnostic.play_once(true));
    CHECK(io.tones == 0);
  }
  {
    // Only the final amplifier readback disagrees: the tone already played, so
    // the result must stay Unknown.
    SpeakerDiagnosticFake io;
    io.regs.fill(0xff);
    io.corrupt_after = 5;
    hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
    CHECK(diagnostic.prepare());
    CHECK(!diagnostic.play_once(true));
    CHECK(io.tones == 1);
    CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Unknown);
  }
  {
    SpeakerDiagnosticFake io;
    io.open_delay = 501;
    hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
    CHECK(!diagnostic.prepare());
    CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Unknown);
  }
  {
    // Cleanup failure after a verified amplifier-off sequence is still Unknown.
    SpeakerDiagnosticFake io;
    hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
    CHECK(diagnostic.prepare());
    io.close_fails = true;
    CHECK(!diagnostic.play_once(true));
    CHECK(io.tones == 1);
    CHECK((io.regs[0x03] & 0x02) == 0x02);
    CHECK((io.regs[0x05] & 0x02) == 0);
    CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Unknown);
  }
}

TEST("Tab5 speaker failed cleanup can leave PA high and must report unknown") {
  SpeakerDiagnosticFake io;
  io.fail_after_tone = true;
  hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
  CHECK(diagnostic.prepare());
  CHECK(!diagnostic.play_once(true));
  CHECK(io.tones == 1);
  CHECK((io.regs[0x05] & 0x02) == 0x02);  // software cannot promise hardware shutdown
  CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Unknown);
  CHECK(io.calls.back() == 4);
}

TEST("Tab5 speaker cannot turn a high latch into an output on bus failure") {
  SpeakerDiagnosticFake io;
  io.regs[0x05] = 0xff;
  io.cleanup_fails = true;
  hgp::Tab5SpeakerDiagnostic<SpeakerDiagnosticFake> diagnostic(io);
  CHECK(!diagnostic.prepare());
  CHECK(io.regs[0x03] == 0);
  CHECK(std::find(io.calls.begin(), io.calls.end(), 0x203) == io.calls.end());
  CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Unknown);
}

TEST("Tab5 USB exchange gates seed on exact ACK and wipes on failure") {
  std::array<uint8_t,56> offer{};
  std::copy_n("HGSEED02", 8, offer.begin());
  for (size_t i=8; i<offer.size(); ++i) offer[i] = static_cast<uint8_t>(i-8);
  const uint8_t expected[32] = {
    0xd4,0x3a,0x4b,0xc0,0x7f,0x02,0x20,0xfc,0xf0,0xf0,0xbf,0x25,0x9d,0xd5,0x79,0xee,
    0x23,0x42,0xba,0x8d,0x4a,0x06,0x76,0x96,0xf9,0xff,0xea,0x49,0xb2,0xe2,0x42,0x19};
  hgp::Tab5OfflineSeed seed;
  hgp::Tab5OfflineExchange exchange(100);
  uint8_t out[32]{};
  CHECK(exchange.feed(offer.data(), 20, 101));
  CHECK(!seed.take(out, sizeof(out)));
  CHECK(exchange.feed(offer.data()+20, 36, 102));
  std::array<uint8_t,56> reply{};
  CHECK(exchange.response(reply.data(), reply.size(), 103));
  CHECK(std::equal(reply.begin(), reply.begin()+8, "HGPROOF2"));
  CHECK(std::equal(reply.begin()+8, reply.begin()+24, offer.begin()+8));
  CHECK(std::equal(reply.begin()+24, reply.end(), expected));
  std::copy_n("HGACK002", 8, reply.begin());
  CHECK(exchange.feed(reply.data(), reply.size(), 104));
  CHECK(!seed.take(out, sizeof(out)));
  CHECK(exchange.finish(seed, 105));
  CHECK(exchange.cleared());
  CHECK(seed.take(out, sizeof(out)));
  CHECK(std::equal(out, out+32, offer.begin()+24));
  CHECK(!exchange.finish(seed, 106));
}

TEST("Tab5 USB exchange rejects malformed partial late and duplicate frames") {
  std::array<uint8_t,57> offer{};
  std::copy_n("HGSEED02", 8, offer.begin());
  for (int mode=0; mode<10; ++mode) {
    hgp::Tab5OfflineSeed seed;
    hgp::Tab5OfflineExchange exchange(0);
    std::array<uint8_t,56> reply{};
    if (mode==0) CHECK(!exchange.feed(offer.data(),57,1));
    if (mode==1) { CHECK(exchange.feed(offer.data(),55,1)); CHECK(!exchange.finish(seed,2)); }
    if (mode==2) CHECK(!exchange.feed(offer.data(),56,60000));
    if (mode>=3) {
      CHECK(exchange.feed(offer.data(),56,1));
      CHECK(exchange.response(reply.data(),56,2));
      std::copy_n("HGACK002",8,reply.begin());
      if (mode==3) { reply[24]^=1; CHECK(!exchange.feed(reply.data(),56,3)); }
      if (mode==4) { CHECK(exchange.feed(reply.data(),55,3)); CHECK(!exchange.finish(seed,4)); }
      if (mode==5) { CHECK(exchange.feed(reply.data(),56,3)); CHECK(!exchange.feed(reply.data(),56,4)); }
      if (mode==6) CHECK(!exchange.finish(seed,60000));
      if (mode==7) { reply[8]^=1; CHECK(!exchange.feed(reply.data(),56,3)); }
      if (mode==8) CHECK(!exchange.feed(offer.data(),56,3));
      if (mode==9) { CHECK(exchange.feed(reply.data(),56,3)); CHECK(!exchange.feed(offer.data(),1,4)); }
    }
    CHECK(exchange.cleared());
    uint8_t out[32]{};
    CHECK(!seed.take(out,32));
  }
}

TEST("Tab5 USB exchange permits intact replay if device challenge repeats after reset") {
  std::array<uint8_t,56> offer{}, ack{};
  std::copy_n("HGSEED02",8,offer.begin());
  for (int boot=0; boot<2; ++boot) {
    hgp::Tab5OfflineSeed seed;
    hgp::Tab5OfflineExchange exchange(0);
    std::array<uint8_t,56> proof{};
    CHECK(exchange.feed(offer.data(),56,1));
    CHECK(exchange.response(proof.data(),56,2));
    if (boot==0) { ack=proof; std::copy_n("HGACK002",8,ack.begin()); }
    CHECK(exchange.feed(ack.data(),56,3)); // historical ACK also accepted on boot 2
    CHECK(exchange.finish(seed,4));
    uint8_t out[32]{};
    CHECK(seed.take(out,32));
  }
}

TEST("Tab5 USB exchange rejects old ACK when device challenge changes") {
  std::array<uint8_t,56> offer{}, ack{}, proof{};
  std::copy_n("HGSEED02",8,offer.begin());
  std::array<uint8_t,16> challenge{};
  hgp::Tab5OfflineExchange first(0, challenge);
  CHECK(first.feed(offer.data(),56,1));
  CHECK(first.response(ack.data(),56,2));
  std::copy_n("HGACK002",8,ack.begin());
  challenge[0] = 1;
  hgp::Tab5OfflineExchange fresh(0, challenge);
  CHECK(fresh.feed(offer.data(),56,1));
  CHECK(fresh.response(proof.data(),56,2));
  CHECK(!fresh.feed(ack.data(),56,3));
  hgp::Tab5OfflineSeed seed;
  CHECK(!fresh.finish(seed,4));
  CHECK(fresh.cleared());
}

TEST("Tab5 USB exchange fails closed before ACK and on invalid offer") {
  for (int mode=0; mode<4; ++mode) {
    hgp::Tab5OfflineExchange exchange(0);
    hgp::Tab5OfflineSeed seed;
    std::array<uint8_t,56> offer{};
    std::copy_n("HGSEED02",8,offer.begin());
    if (mode==0) { offer[0]^=1; CHECK(!exchange.feed(offer.data(),56,1)); }
    if (mode==1) CHECK(!exchange.feed(nullptr,1,1));
    if (mode==2) { CHECK(exchange.feed(offer.data(),56,1)); CHECK(!exchange.finish(seed,2)); }
    if (mode==3) { CHECK(exchange.feed(offer.data(),56,1)); CHECK(!exchange.feed(offer.data(),56,2)); }
    CHECK(exchange.cleared());
    uint8_t out[32]{};
    CHECK(!seed.take(out,32));
    CHECK(!exchange.feed(offer.data(),56,3));
  }
}

TEST("Tab5 offline host seed window is bounded and clock-safe") {
  CHECK(hgp::kTab5OfflineSeedWindowMs == 15000);
  CHECK(hgp::tab5_offline_window_open(1000, 1000));
  CHECK(hgp::tab5_offline_window_open(1000, 15999));
  CHECK(!hgp::tab5_offline_window_open(1000, 16000));
  CHECK(!hgp::tab5_offline_window_open(16000, 1000)); // clock went backwards
}

TEST("Tab5 offline route keeps preview without a seed and hands over after one") {
  using hgp::Tab5OfflineRoute;
  // The decision itself: only a completed seed inside the window reaches the App.
  CHECK(hgp::tab5_offline_route(true, true) == Tab5OfflineRoute::App);
  CHECK(hgp::tab5_offline_route(false, true) == Tab5OfflineRoute::Preview);
  CHECK(hgp::tab5_offline_route(false, false) == Tab5OfflineRoute::Preview);
  CHECK(hgp::tab5_offline_route(true, false) == Tab5OfflineRoute::Preview);

  std::array<uint8_t,56> offer{};
  std::copy_n("HGSEED02", 8, offer.begin());
  for (size_t i=8; i<offer.size(); ++i) offer[i] = static_cast<uint8_t>(i);

  // Nothing arrives: the whole window passes and the preview stays up.
  {
    hgp::Tab5OfflineSeed seed;
    hgp::Tab5OfflineExchange exchange(1000);
    CHECK(!exchange.accepted());
    const bool seeded = exchange.finish(seed, 1000 + hgp::kTab5OfflineSeedWindowMs);
    CHECK(!seeded);
    CHECK(hgp::tab5_offline_route(seeded,
                                  hgp::tab5_offline_window_open(
                                      1000, 1000 + hgp::kTab5OfflineSeedWindowMs)) ==
          Tab5OfflineRoute::Preview);
  }

  // A verified exchange inside the window is the App route.
  {
    hgp::Tab5OfflineSeed seed;
    hgp::Tab5OfflineExchange exchange(1000);
    std::array<uint8_t,56> reply{};
    CHECK(exchange.feed(offer.data(), offer.size(), 1100));
    CHECK(exchange.response(reply.data(), reply.size(), 1200));
    std::copy_n("HGACK002", 8, reply.begin());
    CHECK(exchange.feed(reply.data(), reply.size(), 1300));
    const uint64_t now = 1400;
    const bool seeded = exchange.finish(seed, now);
    CHECK(seeded);
    CHECK(hgp::tab5_offline_route(seeded, hgp::tab5_offline_window_open(1000, now)) ==
          Tab5OfflineRoute::App);
    uint8_t out[32]{};
    CHECK(seed.take(out, sizeof(out)));
  }

  // The same verified seed handed over after the window keeps the preview.
  {
    hgp::Tab5OfflineSeed seed;
    hgp::Tab5OfflineExchange exchange(1000);
    std::array<uint8_t,56> reply{};
    CHECK(exchange.feed(offer.data(), offer.size(), 1100));
    CHECK(exchange.response(reply.data(), reply.size(), 1200));
    std::copy_n("HGACK002", 8, reply.begin());
    CHECK(exchange.feed(reply.data(), reply.size(), 1300));
    const uint64_t now = 1000 + hgp::kTab5OfflineSeedWindowMs;
    const bool seeded = exchange.finish(seed, now);
    CHECK(seeded); // the exchange's own 60 s window still allows the handover
    CHECK(hgp::tab5_offline_route(seeded, hgp::tab5_offline_window_open(1000, now)) ==
          Tab5OfflineRoute::Preview);
  }
}

TEST("Tab5 offline App applies brightness without persistent storage") {
  const uint8_t frame[40] = {'H','G','S','E','E','D','0','1',1};
  hgp::Tab5OfflineSeed seed;
  CHECK(seed.feed(frame, sizeof(frame)));
  hgp::Tab5OfflineSystem system(seed, [] { return uint32_t{100}; }, [] { std::abort(); });
  hgp::Tab5OfflineTransport transport;
  struct Display : hg::Display {
    std::vector<uint16_t> pixels = std::vector<uint16_t>(720 * 1280);
    int brightness = -1;
    hg::DisplayInfo info() const override { hg::DisplayInfo d{}; d.width = 720; d.height = 1280; d.has_backlight = true; return d; }
    uint16_t* framebuffer() override { return pixels.data(); }
    void flush(uint16_t, uint16_t) override {}
    void set_backlight(uint8_t percent) override { brightness = percent; }
  } display;
  hgp::Tab5OfflineStorage storage;
  auto hal = hgp::tab5_offline_hal(system, transport, display, storage);
  hg::App app(hal, hgp::tab5_offline_profile(200));
  app.begin();
  CHECK_EQ(app.console("set brightness 25"), std::string("@ok brightness"));
  CHECK_EQ(display.brightness, 25);
  CHECK(!storage.get("device_key"));
  CHECK_EQ(app.console("get brightness"), std::string("@value {\"key\":\"brightness\",\"value\":\"25\"}"));
  hgp::Tab5OfflineStorage new_session;
  CHECK(!new_session.get("brightness"));
  CHECK(!app.online());
  CHECK(!transport.violated());
}

TEST("Tab5 real App begins and handles touch with ephemeral offline adapters") {
  const uint8_t frame[40] = {'H','G','S','E','E','D','0','1',1}; // test fixture only
  hgp::Tab5OfflineSeed seed;
  CHECK(seed.feed(frame, sizeof(frame)));
  static uint32_t now = 0;
  now = 2000;
  hgp::Tab5OfflineSystem system(seed, [] { return now; }, [] { std::abort(); });
  hgp::Tab5OfflineTransport transport;
  struct Display : hg::Display {
    std::vector<uint16_t> pixels = std::vector<uint16_t>(720 * 1280);
    int flushes = 0;
    hg::DisplayInfo info() const override { hg::DisplayInfo d; d.width=720; d.height=1280; return d; }
    uint16_t* framebuffer() override { return pixels.data(); }
    void flush(uint16_t, uint16_t) override { ++flushes; }
  } display;
  hgp::Tab5OfflineStorage storage;
  auto hal = hgp::tab5_offline_hal(system, transport, display, storage);
  CHECK(hal.storage && !hal.mic && !hal.speaker && !hal.updater && !hal.power);
  auto profile = hgp::tab5_offline_profile(200);
  CHECK(profile.default_server_url.empty() && profile.default_access_token.empty());
  hg::App app(hal, profile);
  hg::TouchGestures gestures(app);
  app.begin();
  CHECK(system.consumed());
  CHECK(!app.device_id().empty());
  CHECK(app.model().link == hg::Link::Offline);
  now = 10000; // let the real App's boot screen finish
  app.tick();
  CHECK(app.screen() == hg::Screen::Offline);
  CHECK(display.flushes > 0);
  CHECK(app.settings_title_hit(360, 10));
  CHECK(!app.settings_open());
  gestures.update(true, 360, 10, now);
  now += 1100;
  gestures.tick(now);
  gestures.update(false, 360, 10, now);
  app.tick();
  CHECK(app.settings_open());
  gestures.update(true, 360, 10, now);
  now += 1100;
  gestures.tick(now);
  gestures.update(false, 360, 10, now);
  app.tick();
  CHECK(!app.settings_open());
  CHECK(!app.online());
  CHECK(!transport.violated());
  uint8_t out[32]{};
  CHECK(!seed.take(out, sizeof(out)));
}

TEST("Tab5 offline transport records forbidden connection and send attempts") {
  hgp::Tab5OfflineTransport transport;
  CHECK(!transport.violated());
  transport.close();
  CHECK(!transport.violated());
  transport.connect("ws://invalid", "");
  CHECK(transport.violated());
}


TEST("Tab5 offline diagnostic transport rejects all sends") {
  hgp::Tab5OfflineTransport transport;
  const uint8_t payload[] = {1, 2, 3};
  transport.connect("wss://example.invalid/gadget", "hermes-gadget.v1");
  CHECK(!transport.send_text("hello"));
  CHECK(!transport.send_binary(payload, sizeof(payload)));
  transport.close();
  CHECK(!transport.send_text("after close"));
}

TEST("Tab5 offline seed rejects malformed frames and consumes valid seed once") {
  const uint8_t frame[40] = {'H','G','S','E','E','D','0','1',1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};
  hgp::Tab5OfflineSeed seed;
  uint8_t out[32] = {};
  CHECK(!seed.take(out, sizeof(out)));
  CHECK(seed.feed(frame, 3));
  CHECK(seed.feed(frame + 3, 37));
  CHECK(!seed.take(out, 31));
  CHECK(seed.take(out, sizeof(out)));
  for (size_t i = 0; i < sizeof(out); ++i) CHECK_EQ(out[i], frame[i + 8]);
  CHECK(!seed.take(out, sizeof(out)));
  CHECK(!seed.feed(frame, sizeof(frame)));
  hgp::Tab5OfflineSeed bad;
  const uint8_t wrong[] = {'X'};
  CHECK(!bad.feed(wrong, sizeof(wrong)));
  CHECK(!bad.feed(frame, sizeof(frame)));
  CHECK(!bad.take(out, sizeof(out)));
  hgp::Tab5OfflineSeed incomplete;
  CHECK(incomplete.feed(frame, 39));
  std::fill(std::begin(out), std::end(out), 0xa5);
  CHECK(!incomplete.take(out, sizeof(out)));
  CHECK(std::all_of(std::begin(out), std::end(out), [](uint8_t b) { return b == 0xa5; }));
  CHECK(!incomplete.feed(frame, 2));
  hgp::Tab5OfflineSeed null_input;
  CHECK(!null_input.feed(nullptr, 1));
  hgp::Tab5OfflineSeed oversized;
  uint8_t excess[41] = {};
  CHECK(!oversized.feed(excess, sizeof(excess)));
  hgp::Tab5OfflineSeed fragmented;
  for (auto byte : frame) CHECK(fragmented.feed(&byte, 1));
  CHECK(!fragmented.take(nullptr, 32));
  CHECK(fragmented.take(out, sizeof(out)));
  hgp::Tab5OfflineSeed late_error;
  CHECK(late_error.feed(frame, 3));
  CHECK(!late_error.feed(wrong, sizeof(wrong)));
  CHECK(!late_error.feed(frame + 3, 37));
}

TEST("Tab5 diagnostic: real offline UI preview and touch feedback") {
  struct PreviewDisplay : hg::Display {
    std::vector<uint16_t> pixels = std::vector<uint16_t>(720 * 1280);
    int flushes = 0;
    hg::DisplayInfo info() const override {
      hg::DisplayInfo d{};
      d.width = 720;
      d.height = 1280;
      return d;
    }
    uint16_t* framebuffer() override { return pixels.data(); }
    void flush(uint16_t, uint16_t) override { ++flushes; }
  } display;
  auto model = hgp::tab5_diagnostic_preview();
  CHECK(model.link == hg::Link::Offline);
  CHECK(model.screen == hg::Screen::Offline);
  CHECK(model.hero);
  CHECK(model.title.find("PREVIEW") != std::string::npos);
  CHECK(model.qr == nullptr);
  hg::Ui ui(display);
  ui.render(model);
  CHECK(display.flushes > 0);
  CHECK(std::any_of(display.pixels.begin(), display.pixels.end(),
                    [&](uint16_t p) { return p != display.pixels.front(); }));
  const auto initial = display.pixels;
  hgp::tab5_diagnostic_touch(model, true, 529, 263);
  CHECK_EQ(model.hint, std::string("Touch down/move x=529 y=263"));
  ui.render(model);
  CHECK(display.pixels != initial);
  const auto pressed = display.pixels;
  hgp::tab5_diagnostic_touch(model, false, 529, 263);
  CHECK_EQ(model.hint, std::string("Touch up x=529 y=263"));
  ui.render(model);
  CHECK(display.pixels != pressed);
  CHECK(model.link == hg::Link::Offline);
  const int flushed = display.flushes;
  ui.render(model);
  CHECK_EQ(display.flushes, flushed);
}

TEST("Tab5 diagnostic: bounded RGB565 bars and white border") {
  std::array<uint16_t, 32> pixels{};
  pixels.front() = pixels.back() = 0x1234;
  CHECK(hgp::tab5_diagnostic_pattern(pixels.data() + 1, 6, 5));
  CHECK_EQ(pixels.front(), 0x1234);
  CHECK_EQ(pixels.back(), 0x1234);
  CHECK_EQ(pixels[1], 0xffff);
  CHECK_EQ(pixels[8], 0xf800);
  CHECK_EQ(pixels[9], 0x07e0);
  CHECK_EQ(pixels[11], 0x001f);
  CHECK_EQ(pixels[30], 0xffff);
  CHECK(!hgp::tab5_diagnostic_pattern(nullptr, 6, 5));
  CHECK(!hgp::tab5_diagnostic_pattern(pixels.data(), 0, 5));
  CHECK(!hgp::tab5_diagnostic_pattern(pixels.data(), 6, 0));
  CHECK(hgp::tab5_diagnostic_pattern(pixels.data(), 1, 1));
  CHECK_EQ(pixels[0], 0xffff);
}

TEST("Tab5 MIC probe: silence, a tone and clipping use the shared level math") {
  // The probe reports the same numbers the App's VAD sees: hg::rms/level_percent.
  std::array<int16_t, 240> block{};
  hgp::Tab5MicStats silence;
  CHECK_EQ(hgp::tab5_mic_to_mono(block.data(), block.size(), 1, block.data(), block.size()),
           block.size());
  silence.add(block.data(), block.size());
  CHECK_EQ(silence.rms(), 0.0f);
  CHECK_EQ(silence.peak, 0);
  CHECK_EQ(silence.peak_percent(), uint8_t{0});
  CHECK_EQ(silence.clipped_blocks, 0u);
  CHECK_EQ(silence.frames, uint64_t{240});
  std::array<char, 64> line{};
  const size_t n = hgp::tab5_mic_level_line(line.data(), line.size(), 0);
  CHECK(std::string(line.data(), n) == std::string("mic   0% ------------"));

  // A full-scale square wave at 8192 is -12 dBFS: the meter must equal the App's.
  std::array<int16_t, 240> tone{};
  for (size_t i = 0; i < tone.size(); ++i) tone[i] = i % 2 ? -8192 : 8192;
  hgp::Tab5MicStats voiced;
  voiced.add(tone.data(), tone.size());
  CHECK_EQ(voiced.rms(), 8192.0f);
  CHECK_EQ(voiced.peak, 8192);
  CHECK_EQ(voiced.peak_percent(), hg::level_percent(8192.0f));
  CHECK_EQ(voiced.clipped_blocks, 0u);
  CHECK_EQ(voiced.frames, uint64_t{240});

  // Clipping counts whole blocks that reach full scale, and the peak is clamped.
  std::array<int16_t, 240> loud{};
  for (size_t i = 0; i < loud.size(); ++i) loud[i] = i % 2 ? int16_t(-32768) : int16_t(32767);
  hgp::Tab5MicStats clipped;
  clipped.add(loud.data(), loud.size());
  CHECK_EQ(clipped.peak, 32767);
  CHECK_EQ(clipped.peak_percent(), hg::level_percent(32767.0f));
  CHECK_EQ(clipped.clipped_blocks, 1u);
  CHECK_EQ(clipped.blocks, 1u);
  // The threshold is full scale: one sample short still counts as unclipped.
  std::array<int16_t, 240> near{};
  near.fill(int16_t(hgp::kTab5MicClipLevel - 1));
  hgp::Tab5MicStats soft;
  soft.add(near.data(), near.size());
  CHECK_EQ(soft.clipped_blocks, 0u);
  CHECK_EQ(soft.peak, hgp::kTab5MicClipLevel - 1);
}

TEST("Tab5 MIC probe: session and report windows are bounded and clock-safe") {
  CHECK_EQ(hgp::kTab5MicSessionMs, 30000u);
  CHECK_EQ(hgp::kTab5MicReportMs, 250u);
  CHECK_EQ(hgp::kTab5MicRate, 24000u);
  CHECK(hgp::tab5_mic_session_open(1000, 1000));
  CHECK(hgp::tab5_mic_session_open(1000, 1000 + hgp::kTab5MicSessionMs - 1));
  CHECK(!hgp::tab5_mic_session_open(1000, 1000 + hgp::kTab5MicSessionMs));
  CHECK(!hgp::tab5_mic_session_open(1000, 999));  // clock went backwards

  uint64_t last = 0;
  CHECK(!hgp::tab5_mic_report_due(hgp::kTab5MicReportMs - 1, last));
  CHECK_EQ(last, uint64_t{0});  // a suppressed report never moves the deadline
  CHECK(hgp::tab5_mic_report_due(hgp::kTab5MicReportMs, last));
  CHECK_EQ(last, uint64_t(hgp::kTab5MicReportMs));
  CHECK(!hgp::tab5_mic_report_due(hgp::kTab5MicReportMs + 100, last));
  CHECK(hgp::tab5_mic_report_due(2 * hgp::kTab5MicReportMs, last));
  CHECK(!hgp::tab5_mic_report_due(1, last));  // clock went backwards
  CHECK_EQ(last, uint64_t(2 * hgp::kTab5MicReportMs));
}

TEST("Tab5 MIC probe: downmix and line writing never overrun their buffers") {
  // MIC1/MIC2 arrive interleaved; the probe reduces them to one mono block.
  const std::array<int16_t, 6> stereo{1000, 2000, -3, -2, 32767, 32767};
  std::array<int16_t, 3> mono{};
  CHECK_EQ(hgp::tab5_mic_to_mono(stereo.data(), 3, 2, mono.data(), mono.size()), size_t{3});
  CHECK_EQ(mono[0], int16_t(1500));
  CHECK_EQ(mono[1], int16_t(-2));
  CHECK_EQ(mono[2], int16_t(32767));
  const std::array<int16_t, 8> quad{100, 200, 300, 400, 4, 8, 12, 16};
  CHECK_EQ(hgp::tab5_mic_to_mono(quad.data(), 2, 4, mono.data(), mono.size()), size_t{2});
  CHECK_EQ(mono[0], int16_t(250));
  CHECK_EQ(mono[1], int16_t(10));
  // The caller's capacity caps whole frames only, and nothing is read past the end.
  CHECK_EQ(hgp::tab5_mic_to_mono(stereo.data(), 3, 2, mono.data(), 2), size_t{2});
  CHECK_EQ(hgp::tab5_mic_to_mono(stereo.data(), 3, 2, mono.data(), 0), size_t{0});
  CHECK_EQ(hgp::tab5_mic_to_mono(stereo.data(), 0, 2, mono.data(), mono.size()), size_t{0});
  CHECK_EQ(hgp::tab5_mic_to_mono(stereo.data(), 3, 0, mono.data(), mono.size()), size_t{0});
  CHECK_EQ(hgp::tab5_mic_to_mono(nullptr, 3, 2, mono.data(), mono.size()), size_t{0});
  CHECK_EQ(hgp::tab5_mic_to_mono(stereo.data(), 3, 2, nullptr, mono.size()), size_t{0});

  // A line is either complete or refused: never silently truncated.
  std::array<char, 64> line{};
  const size_t full = hgp::tab5_mic_level_line(line.data(), line.size(), 100);
  CHECK(std::string(line.data(), full) == std::string("mic 100% ############"));
  const size_t half = hgp::tab5_mic_level_line(line.data(), line.size(), 42);
  CHECK(std::string(line.data(), half) == std::string("mic  42% #####-------"));
  std::array<char, 32> exact{};
  CHECK_EQ(hgp::tab5_mic_level_line(exact.data(), half + 1, 42), half);  // length plus the NUL
  CHECK_EQ(hgp::tab5_mic_level_line(exact.data(), half, 42), size_t{0});
  CHECK_EQ(hgp::tab5_mic_level_line(exact.data(), 0, 42), size_t{0});
  CHECK_EQ(hgp::tab5_mic_level_line(nullptr, 32, 42), size_t{0});

  hgp::Tab5MicStats stats;
  std::array<int16_t, 240> block{};
  block.fill(int16_t(-32768));
  stats.add(block.data(), block.size());
  std::array<char, 128> summary_line{};
  const size_t summary = hgp::tab5_mic_summary_line(summary_line.data(), summary_line.size(), stats);
  CHECK(std::string(summary_line.data(), summary) ==
        std::string("mic done: peak 32767 rms 32768.0 max-block-rms 32768.0 frames 240 "
                    "blocks 1 clipped 1"));
  CHECK_EQ(hgp::tab5_mic_summary_line(summary_line.data(), summary, stats), size_t{0});
  CHECK_EQ(hgp::tab5_mic_summary_line(summary_line.data(), summary + 1, stats), summary);
  CHECK_EQ(hgp::tab5_mic_summary_line(nullptr, 64, stats), size_t{0});
}

TEST("Tab5 MIC probe: RX-only pin map matches the BSP audio facts") {
  const auto pins = hgp::tab5_mic_pins();
  CHECK_EQ(pins.mclk, 30);
  CHECK_EQ(pins.bclk, 27);
  CHECK_EQ(pins.ws, 29);
  CHECK_EQ(pins.din, 28);
  // 24 kHz is CodecAudio::kRate, the rate the SDK's codec boards run at.
  const auto fs = hgp::tab5_mic_format();
  CHECK_EQ(fs.sample_rate, hgp::kTab5MicRate);
  CHECK_EQ(fs.channels, 2);  // MIC1|MIC2 as 16-bit slots; the probe downmixes
  CHECK_EQ(fs.bits, 16);
}

TEST("Tab5 audio UI: the panel splits into a tone half and a mic half") {
  using Target = hgp::Tab5AudioTarget;
  CHECK(hgp::tab5_audio_target_at(360, 0, 720, 1280) == Target::Tone);
  CHECK(hgp::tab5_audio_target_at(0, 639, 720, 1280) == Target::Tone);
  CHECK(hgp::tab5_audio_target_at(719, 640, 720, 1280) == Target::Mic);
  CHECK(hgp::tab5_audio_target_at(360, 1279, 720, 1280) == Target::Mic);
  // Outside the panel, and a panel without rows, has no target: never a stray tone.
  CHECK(hgp::tab5_audio_target_at(-1, 100, 720, 1280) == Target::None);
  CHECK(hgp::tab5_audio_target_at(720, 100, 720, 1280) == Target::None);
  CHECK(hgp::tab5_audio_target_at(10, -1, 720, 1280) == Target::None);
  CHECK(hgp::tab5_audio_target_at(10, 1280, 720, 1280) == Target::None);
  CHECK(hgp::tab5_audio_target_at(0, 0, 0, 1280) == Target::None);
  CHECK(hgp::tab5_audio_target_at(0, 0, 720, 0) == Target::None);
  // Odd heights: the middle row belongs to the microphone, never to the tone.
  CHECK(hgp::tab5_audio_target_at(0, 3, 10, 9) == Target::Tone);
  CHECK(hgp::tab5_audio_target_at(0, 4, 10, 9) == Target::Mic);
}

TEST("Tab5 audio UI: the lines name the held tone state and the measured level") {
  // The state line is covered where the confirmed convention is asserted; here
  // only the microphone value keeps its own shape.
  auto mic = [](int pct) {
    std::array<char, 32> buffer{};
    const size_t n = hgp::tab5_audio_mic_line(buffer.data(), buffer.size(), pct);
    return std::string(buffer.data(), n);
  };
  CHECK_EQ(mic(-1), std::string("MIC: --"));
  CHECK_EQ(mic(-100), std::string("MIC: --"));
  CHECK_EQ(mic(0), std::string("MIC: 0%"));
  CHECK_EQ(mic(33), std::string("MIC: 33%"));
  CHECK_EQ(mic(100), std::string("MIC: 100%"));
  CHECK_EQ(mic(101), std::string("MIC: 100%"));  // clamped, an impossible level is never printed
  std::array<char, 32> mic_line{};
  const size_t mic_need = hgp::tab5_audio_mic_line(mic_line.data(), mic_line.size(), 33);
  CHECK_EQ(hgp::tab5_audio_mic_line(mic_line.data(), mic_need, 33), size_t{0});
  CHECK_EQ(hgp::tab5_audio_mic_line(mic_line.data(), mic_need + 1, 33), mic_need);
  CHECK_EQ(hgp::tab5_audio_mic_line(nullptr, 32, 33), size_t{0});
  CHECK_EQ(hgp::tab5_audio_mic_line(mic_line.data(), 0, 33), size_t{0});

  // The window's result is the same percentage the App's VAD and the mic probe use.
  hgp::Tab5MicStats empty;
  CHECK_EQ(hgp::tab5_audio_mic_result(empty), -1);  // nothing measured yet
  std::array<int16_t, 240> block{};
  for (size_t i = 0; i < block.size(); ++i) block[i] = i % 2 ? int16_t(-8192) : int16_t(8192);
  hgp::Tab5MicStats stats;
  stats.add(block.data(), block.size());
  CHECK_EQ(hgp::tab5_audio_mic_result(stats), int(hg::level_percent(8192.0f)));
}

TEST("Tab5 audio UI: microphone windows are short, explicit and bounded") {
  static_assert(hgp::kTab5AudioMicWindowMs >= 2000, "long enough to say something");
  static_assert(hgp::kTab5AudioMicWindowMs <= 8000, "short: no continuous capture");
  CHECK(hgp::tab5_audio_mic_window_open(1000, 1000));
  CHECK(hgp::tab5_audio_mic_window_open(1000, 1000 + hgp::kTab5AudioMicWindowMs - 1));
  CHECK(!hgp::tab5_audio_mic_window_open(1000, 1000 + hgp::kTab5AudioMicWindowMs));
  CHECK(!hgp::tab5_audio_mic_window_open(1000, 999));  // clock went backwards
}

TEST("Tab5 audio UI: a tap plays one tone and the screen holds what happened") {
  struct PreviewDisplay : hg::Display {
    std::vector<uint16_t> pixels = std::vector<uint16_t>(720 * 1280);
    int flushes = 0;
    hg::DisplayInfo info() const override {
      hg::DisplayInfo d{};
      d.width = 720;
      d.height = 1280;
      return d;
    }
    uint16_t* framebuffer() override { return pixels.data(); }
    void flush(uint16_t, uint16_t) override { ++flushes; }
  } display;
  hgp::Tab5AudioScreen screen(720, 1280);
  const hg::UiModel& model = screen.model();
  CHECK_EQ(model.title, std::string("Hermes - TAB5 AUDIO"));
  CHECK(model.hero);
  CHECK(model.link == hg::Link::Offline);
  CHECK_EQ(model.headline, std::string("TONE: ready"));  // silence at boot
  CHECK_EQ(model.hint, std::string("TAP TOP: tone   TAP BOTTOM: mic level"));
  CHECK_EQ(model.detail.substr(0, 7), std::string("MIC: --"));
  CHECK(model.detail.find("TAP TOP: one 250 ms tone") != std::string::npos);
  CHECK(model.detail.find("TAP BOTTOM: mic level") != std::string::npos);
  CHECK(model.detail.find("TONES PLAYED: 0") != std::string::npos);
  CHECK(model.detail.find("LAST:") == std::string::npos);  // no outcome yet
  CHECK(model.detail.find("do not assume") == std::string::npos);
  CHECK_EQ(int(model.caption_lines), 3);
  hg::Ui ui(display);
  ui.render(model);
  CHECK(display.flushes > 0);
  CHECK(std::any_of(display.pixels.begin(), display.pixels.end(),
                    [&](uint16_t p) { return p != display.pixels.front(); }));

  // One press is one action: a move and the lift never repeat it.
  CHECK(screen.touch(true, 360, 200, 1000) == hgp::Tab5AudioAction::PlayTone);
  CHECK(screen.touch(true, 361, 640, 1010) == hgp::Tab5AudioAction::None);  // same contact
  CHECK(screen.touch(false, 361, 640, 1020) == hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.model().headline, std::string("TONE: playing (amplifier on)"));
  CHECK(screen.tone_phase() == hgp::Tab5AudioTonePhase::Playing);
  CHECK_EQ(screen.tones_played(), 1u);
  ui.render(screen.model());
  // The play line stays readable before any result may replace it ...
  screen.tone_done(hgp::Tab5SpeakerState::SafeOff, 1300);
  CHECK_EQ(screen.model().headline, std::string("TONE: playing (amplifier on)"));
  CHECK(!screen.tick(2499));  // 1499 ms after the tap: still held
  CHECK(screen.tick(2500));   // 1500 ms: the result appears
  CHECK_EQ(screen.model().headline, std::string("TONE: off and verified"));
  CHECK(screen.model().detail.find("TONES PLAYED: 1   LAST: verified") != std::string::npos);
  const auto drawn = display.pixels;
  ui.render(screen.model());
  CHECK(display.pixels != drawn);  // the result reached the panel, not just the log
  // ... and the result is held too before the screen is ready again.
  CHECK(!screen.tick(3999));
  CHECK(screen.tick(4000));
  CHECK_EQ(screen.model().headline, std::string("TONE: ready"));
  CHECK(screen.model().detail.find("LAST: verified") != std::string::npos);

  // The microphone half opens one bounded window, and nothing else runs in it.
  CHECK(screen.touch(true, 360, 1000, 8000) == hgp::Tab5AudioAction::MeasureMic);
  CHECK(screen.mic_active());
  CHECK(screen.model().screen == hg::Screen::Listening);
  CHECK_EQ(screen.model().detail.substr(0, 14), std::string("MIC: listening"));
  CHECK(screen.touch(true, 360, 1000, 8100) == hgp::Tab5AudioAction::None);  // still down
  CHECK(screen.touch(false, 360, 1000, 8200) == hgp::Tab5AudioAction::None);
  CHECK(screen.touch(true, 100, 300, 8300) == hgp::Tab5AudioAction::None);  // no tone while capturing
  CHECK(screen.touch(false, 100, 300, 8350) == hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.tones_played(), 1u);  // the refused tap changed nothing
  CHECK(screen.mic_level(42));
  CHECK_EQ(screen.model().level, uint8_t{42});
  CHECK_EQ(screen.model().detail.substr(0, 8), std::string("MIC: 42%"));
  // The window's own bound ends it, and the peak block is the reported result.
  CHECK(!hgp::tab5_audio_mic_window_open(8000, 8000 + hgp::kTab5AudioMicWindowMs));
  hgp::Tab5MicStats stats;
  std::array<int16_t, 240> block{};
  for (size_t i = 0; i < block.size(); ++i) block[i] = i % 2 ? int16_t(-8192) : int16_t(8192);
  stats.add(block.data(), block.size());
  screen.mic_done(stats);
  CHECK(!screen.mic_active());
  CHECK(screen.model().screen == hg::Screen::Offline);
  CHECK_EQ(screen.model().level, uint8_t{0});
  const std::string result =
      std::string("MIC: ") + std::to_string(int(hg::level_percent(8192.0f))) + "%";
  CHECK_EQ(screen.model().detail.substr(0, result.size()), result);
  CHECK(!screen.mic_level(50));  // a closed window takes no more levels
  // The microphone keeps working after a tone, and the tone keeps working after it.
  CHECK(screen.touch(true, 360, 1000, 9000) == hgp::Tab5AudioAction::MeasureMic);
  CHECK(screen.touch(false, 360, 1000, 9100) == hgp::Tab5AudioAction::None);
  screen.mic_done(hgp::Tab5MicStats{});
  CHECK(screen.touch(true, 360, 200, 13000) == hgp::Tab5AudioAction::PlayTone);
  CHECK(screen.touch(false, 360, 200, 13100) == hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.tones_played(), 2u);
}

TEST("Tab5 audio UI: every failure is on screen and no target repeats it") {
  hgp::Tab5AudioScreen screen(720, 1280);
  screen.set_mic_fault("the ES7210 did not start");
  CHECK_EQ(screen.model().detail.substr(0, 5), std::string("MIC: "));
  CHECK(screen.model().detail.find("the ES7210 did not start") != std::string::npos);
  // A failed microphone path cannot be retried into an unknown codec state.
  CHECK(screen.touch(true, 360, 1000, 1000) == hgp::Tab5AudioAction::None);
  CHECK(screen.touch(false, 360, 1000, 1100) == hgp::Tab5AudioAction::None);
  // The tone path is separate and reports the state machine's own word.
  CHECK(screen.touch(true, 360, 200, 1200) == hgp::Tab5AudioAction::PlayTone);
  CHECK(screen.touch(false, 360, 200, 1300) == hgp::Tab5AudioAction::None);
  screen.tone_done(hgp::Tab5SpeakerState::Unknown, 1400);
  CHECK(screen.tick(2800));
  CHECK_EQ(screen.model().headline, std::string("TONE: unknown"));
  CHECK(screen.model().detail.find("LAST: UNKNOWN") != std::string::npos);
  CHECK(screen.model().detail.find("do not assume") != std::string::npos);
  CHECK_EQ(int(screen.model().caption_lines), 4);

  // An audio path that never started shows why, and neither half can run.
  hgp::Tab5AudioScreen dead(720, 1280);
  dead.set_audio_fault("I2S_NUM_1 transmit channel failed");
  CHECK_EQ(dead.model().detail.substr(0, 7), std::string("AUDIO: "));
  CHECK(dead.model().detail.find("I2S_NUM_1 transmit channel failed") != std::string::npos);
  CHECK(dead.touch(true, 360, 200, 1000) == hgp::Tab5AudioAction::None);
  CHECK(dead.touch(false, 360, 200, 1010) == hgp::Tab5AudioAction::None);
  CHECK(dead.touch(true, 360, 1000, 1020) == hgp::Tab5AudioAction::None);
  CHECK_EQ(dead.tones_played(), 0u);
  CHECK(!dead.mic_active());

  // Without touch the mode cannot be reached at all, and the screen says so.
  hgp::Tab5AudioScreen mute(720, 1280);
  mute.set_input_fault("touch unavailable");
  CHECK_EQ(mute.model().detail.substr(0, 7), std::string("INPUT: "));
  CHECK(mute.model().detail.find("touch unavailable") != std::string::npos);
  CHECK(mute.touch(true, 360, 200, 1000) == hgp::Tab5AudioAction::None);
  CHECK(mute.touch(false, 360, 200, 1010) == hgp::Tab5AudioAction::None);
  CHECK_EQ(mute.tones_played(), 0u);
}

TEST("Tab5 audio UI: an audible but safe tone on the confirmed enable convention") {
  // About -12 dBFS: 32768 * 10^(-12/20) = 8212, so 8190 sits just under it and
  // still leaves clipping headroom for the square wave's other half.
  CHECK_EQ(int(hgp::kTab5AudioTonePeak), 8190);
  const double dbfs = 20.0 * std::log10(double(hgp::kTab5AudioTonePeak) / 32768.0);
  CHECK(dbfs > -13.0 && dbfs < -11.0);
  CHECK(hgp::kTab5AudioTonePeak < 32767);
  CHECK(hgp::kTab5AudioTonePeak > hgp::kTab5SpeakerTonePeak);  // louder than the quiet probe
  CHECK_EQ(int(hgp::kTab5AudioVolume), 75);
  CHECK(hgp::kTab5AudioVolume <= 100);  // esp_codec_dev volume is a percentage
  CHECK(hgp::kTab5AudioVolume > 50);    // clearly above the conservative speaker probe
  // The duration and the 1 kHz shape did not change, and neither did the quiet
  // speaker-only probe's own amplitude.
  CHECK_EQ(hgp::kTab5SpeakerToneHz, 1000u);
  CHECK_EQ(hgp::kTab5SpeakerToneMs, 250u);
  CHECK_EQ(hgp::kTab5SpeakerToneFrames, size_t{6000});
  std::array<int16_t, hgp::kTab5SpeakerToneFrames> pcm{};
  CHECK_EQ(hgp::tab5_speaker_tone(pcm.data(), pcm.size(), hgp::kTab5AudioTonePeak), pcm.size());
  CHECK_EQ(pcm[0], hgp::kTab5AudioTonePeak);
  CHECK_EQ(pcm[11], hgp::kTab5AudioTonePeak);
  CHECK_EQ(pcm[12], int16_t(-hgp::kTab5AudioTonePeak));
  CHECK_EQ(pcm[pcm.size() - 1], int16_t(-hgp::kTab5AudioTonePeak));
  for (size_t i = 0; i < pcm.size(); ++i)
    CHECK(pcm[i] == (i % 24 < 12 ? hgp::kTab5AudioTonePeak : -hgp::kTab5AudioTonePeak));
  CHECK_EQ(hgp::tab5_speaker_tone(pcm.data(), 4, 0), size_t{0});
  CHECK_EQ(hgp::tab5_speaker_tone(nullptr, 4, 100), size_t{0});
  std::array<int16_t, 8> quiet{};
  CHECK_EQ(hgp::tab5_speaker_tone(quiet.data(), quiet.size()), quiet.size());
  CHECK_EQ(quiet[0], hgp::kTab5SpeakerTonePeak);

  // The enable convention measured by ear on this unit: HIGH was audible, LOW
  // was silent. The default path is the confirmed one; the inverted level is
  // still expressible, it is just never played automatically any more.
  CHECK(hgp::kTab5AudioConfirmedPa == hgp::Tab5AudioPa::High);
  CHECK(hgp::kTab5SpeakerConfirmedDrive == hgp::Tab5SpeakerPa::High);
  CHECK_EQ(std::string(hgp::tab5_audio_pa_text(hgp::kTab5AudioConfirmedPa)),
           std::string("pin HIGH"));
  CHECK(hgp::kTab5AudioInvertedPa == hgp::Tab5AudioPa::Low);
  CHECK_EQ(std::string(hgp::tab5_audio_pa_text(hgp::kTab5AudioInvertedPa)),
           std::string("pin LOW"));

  // The exact lines the panel shows, complete or refused, never truncated into
  // a different state.
  auto status = [](hgp::Tab5AudioTonePhase phase) {
    std::array<char, 40> buffer{};
    const size_t n = hgp::tab5_audio_status_line(buffer.data(), buffer.size(), phase);
    return std::string(buffer.data(), n);
  };
  CHECK_EQ(status(hgp::Tab5AudioTonePhase::Ready), std::string("TONE: ready"));
  CHECK_EQ(status(hgp::Tab5AudioTonePhase::Playing), std::string("TONE: playing (amplifier on)"));
  CHECK_EQ(status(hgp::Tab5AudioTonePhase::Verified), std::string("TONE: off and verified"));
  CHECK_EQ(status(hgp::Tab5AudioTonePhase::Unknown), std::string("TONE: unknown"));
  std::array<char, 40> line{};
  const size_t need =
      hgp::tab5_audio_status_line(line.data(), line.size(), hgp::Tab5AudioTonePhase::Verified);
  CHECK_EQ(hgp::tab5_audio_status_line(line.data(), need, hgp::Tab5AudioTonePhase::Verified),
           size_t{0});
  CHECK_EQ(hgp::tab5_audio_status_line(line.data(), need + 1, hgp::Tab5AudioTonePhase::Verified),
           need);
  CHECK_EQ(hgp::tab5_audio_status_line(nullptr, 40, hgp::Tab5AudioTonePhase::Verified), size_t{0});
  CHECK_EQ(hgp::tab5_audio_status_line(line.data(), 0, hgp::Tab5AudioTonePhase::Verified), size_t{0});

  auto hint = [] {
    std::array<char, 64> buffer{};
    const size_t n = hgp::tab5_audio_hint_line(buffer.data(), buffer.size());
    return std::string(buffer.data(), n);
  };
  CHECK_EQ(hint(), std::string("TAP TOP: one 250 ms tone   TAP BOTTOM: mic level"));
  auto bar = [] {
    std::array<char, 48> buffer{};
    const size_t n = hgp::tab5_audio_bar_hint(buffer.data(), buffer.size());
    return std::string(buffer.data(), n);
  };
  CHECK_EQ(bar(), std::string("TAP TOP: tone   TAP BOTTOM: mic level"));

  auto counter = [](unsigned tones, hgp::Tab5AudioTonePhase last) {
    std::array<char, 64> buffer{};
    const size_t n = hgp::tab5_audio_counter_line(buffer.data(), buffer.size(), tones, last);
    return std::string(buffer.data(), n);
  };
  CHECK_EQ(counter(0, hgp::Tab5AudioTonePhase::Ready), std::string("TONES PLAYED: 0"));
  // Only a tone's own outcome is named, never a phase that is not one.
  CHECK_EQ(counter(0, hgp::Tab5AudioTonePhase::Unknown), std::string("TONES PLAYED: 0"));
  CHECK_EQ(counter(3, hgp::Tab5AudioTonePhase::Playing), std::string("TONES PLAYED: 3"));
  CHECK_EQ(counter(1, hgp::Tab5AudioTonePhase::Verified),
           std::string("TONES PLAYED: 1   LAST: verified"));
  CHECK_EQ(counter(3, hgp::Tab5AudioTonePhase::Unknown),
           std::string("TONES PLAYED: 3   LAST: UNKNOWN"));
  std::array<char, 64> counted{};
  const size_t counted_need = hgp::tab5_audio_counter_line(
      counted.data(), counted.size(), 3, hgp::Tab5AudioTonePhase::Unknown);
  CHECK_EQ(hgp::tab5_audio_counter_line(counted.data(), counted_need, 3,
                                        hgp::Tab5AudioTonePhase::Unknown),
           size_t{0});
  CHECK_EQ(hgp::tab5_audio_counter_line(counted.data(), counted_need + 1, 3,
                                        hgp::Tab5AudioTonePhase::Unknown),
           counted_need);
  CHECK_EQ(hgp::tab5_audio_counter_line(nullptr, 64, 3, hgp::Tab5AudioTonePhase::Unknown),
           size_t{0});

  // The cooldown that refuses a queued tone: never on the first tap, never
  // inside the window, and never on a backwards clock.
  CHECK_EQ(hgp::kTab5AudioToneCooldownMs, 1000u);
  CHECK(hgp::kTab5AudioToneCooldownMs > hgp::kTab5SpeakerToneMs);
  CHECK(hgp::kTab5AudioStatusHoldMs >= 1500u);
  CHECK(hgp::tab5_audio_tone_ready(1000, 2000));
  CHECK(!hgp::tab5_audio_tone_ready(1000, 1999));
  CHECK(!hgp::tab5_audio_tone_ready(1000, 999));  // clock went backwards
}

TEST("Tab5 speaker probe: the explicit A/B polarity probe stays expressible, never automatic") {
  struct ProbeFake {
    std::array<uint8_t, 256> regs{};
    std::vector<uint8_t> config_writes;  // every 0x05 value written, in order
    std::vector<bool> high_at_tone;      // 0x05 pin 1 while each tone was written
    std::vector<bool> mutes;
    unsigned opens = 0, closes = 0;
    uint8_t volume = 0;
    bool opened_muted = false;
    uint64_t time = 0;
    uint64_t now_ms() const { return time; }
    bool read(uint8_t reg, uint8_t& value, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 50);
      time += 1;
      value = regs[reg];
      return true;
    }
    bool write(uint8_t reg, uint8_t value, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 50);
      time += 1;
      if (reg == hgp::kTab5SpeakerConfigReg) config_writes.push_back(value);
      regs[reg] = value;
      return true;
    }
    bool open_dac(uint8_t v, bool muted, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 500);
      CHECK(v > 0 && v <= 100);
      CHECK(muted);  // nothing is audible before the attempt's own unmute
      // The reviewed off state was requested and read back first: pin low,
      // push-pull, an output.
      CHECK_EQ(int(regs[0x05] & 0x02), 0);
      CHECK_EQ(int(regs[0x07] & 0x02), 0);
      CHECK_EQ(int(regs[0x03] & 0x02), 0x02);
      volume = v;
      opened_muted = muted;
      ++opens;
      time += 5;
      return true;
    }
    bool set_mute(bool muted, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 50);
      mutes.push_back(muted);
      time += 1;
      return true;
    }
    bool tone(const int16_t* pcm, size_t frames, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 1000);
      CHECK_EQ(frames, hgp::kTab5SpeakerToneFrames);
      for (size_t i = 0; i < frames; ++i)
        CHECK(pcm[i] == (i % 24 < 12 ? hgp::kTab5AudioTonePeak : -hgp::kTab5AudioTonePeak));
      high_at_tone.push_back((regs[hgp::kTab5SpeakerConfigReg] & hgp::kTab5SpeakerMask) != 0);
      time += 250;
      return true;
    }
    bool close_dac(uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 500);
      ++closes;
      time += 1;
      return true;
    }
  };

  ProbeFake io;
  // Unrelated bits of every touched register must survive the probe.
  io.regs[0x05] = 0xA5;
  io.regs[0x07] = 0x0F;
  io.regs[0x03] = 0x0F;
  std::vector<hgp::Tab5SpeakerPa> announced;
  struct Ctx { std::vector<hgp::Tab5SpeakerPa>* out; } ctx{&announced};
  auto hook = [](void* raw, hgp::Tab5SpeakerPa pa) { static_cast<Ctx*>(raw)->out->push_back(pa); };
  hgp::Tab5SpeakerDiagnostic<ProbeFake> probe(io);
  // Nothing plays before preparation, and a refused call spends nothing.
  CHECK(!probe.play_probe(true, hgp::kTab5AudioTonePeak, hook, &ctx));
  CHECK(probe.state() == hgp::Tab5SpeakerState::Unknown);
  CHECK(probe.prepare(hgp::kTab5AudioVolume));
  CHECK_EQ(int(io.volume), int(hgp::kTab5AudioVolume));
  CHECK(io.opened_muted);
  // An absent explicit action or an impossible amplitude plays nothing.
  CHECK(!probe.play_probe(false, hgp::kTab5AudioTonePeak, hook, &ctx));
  CHECK(!probe.play_probe(true, 0, hook, &ctx));
  CHECK(!probe.play_probe(true, -8190, hook, &ctx));
  CHECK(announced.empty());
  CHECK_EQ(io.config_writes.size(), size_t{1});  // only prepare()'s own off sequence
  CHECK(probe.play_probe(true, hgp::kTab5AudioTonePeak, hook, &ctx));
  CHECK(probe.state() == hgp::Tab5SpeakerState::SafeOff);
  // Two tones, in order: A with the amplifier control HIGH, B with it LOW.
  CHECK_EQ(io.high_at_tone.size(), size_t{2});
  CHECK(io.high_at_tone == std::vector<bool>({true, false}));
  CHECK(announced == std::vector<hgp::Tab5SpeakerPa>({hgp::Tab5SpeakerPa::High,
                                                     hgp::Tab5SpeakerPa::Low}));
  CHECK_EQ(io.closes, 1u);
  CHECK(io.mutes == std::vector<bool>({false, true, false, true}));
  // Only pin 1's bit ever changed in the config register.
  CHECK(!io.config_writes.empty());
  for (uint8_t value : io.config_writes) CHECK_EQ(int(value & ~0x02), int(0xA5 & ~0x02));
  // The reviewed off state is left behind: pin low, push-pull, output;
  // the other registers keep their unrelated bits.
  CHECK_EQ(int(io.regs[0x05] & 0x02), 0);
  CHECK_EQ(int(io.regs[0x07] & 0x02), 0);
  CHECK_EQ(int(io.regs[0x03] & 0x02), 0x02);
  CHECK_EQ(int(io.regs[0x05] & ~0x02), int(0xA5 & ~0x02));
  CHECK_EQ(int(io.regs[0x07] & ~0x02), 0x0F & ~0x02);
  CHECK_EQ(int(io.regs[0x03] & ~0x02), 0x0F & ~0x02);
  CHECK(io.mutes.back());  // muted before the DAC is released
  // One probe per boot: a second call writes nothing and plays nothing.
  const size_t writes = io.config_writes.size();
  CHECK(!probe.play_probe(true, hgp::kTab5AudioTonePeak, hook, &ctx));
  CHECK_EQ(io.config_writes.size(), writes);
  CHECK_EQ(io.high_at_tone.size(), size_t{2});
  CHECK_EQ(io.closes, 1u);
}

TEST("Tab5: entropy window closes after successful or failed seed") {
  for (bool success : {false, true}) {
    std::vector<int> calls;
    CHECK(hgp::tab5_seed_entropy(
        [&] { calls.push_back(1); },
        [&] { calls.push_back(2); return success; },
        [&] { calls.push_back(3); }) == success);
    CHECK(calls == std::vector<int>({1, 2, 3}));
  }
}

TEST("Tab5 audio UI: taps repeat after a cooldown and the counter follows every one") {
  hgp::Tab5AudioScreen screen(720, 1280);
  // Nothing plays before a tap, and nothing has been counted yet.
  CHECK(screen.tone_phase() == hgp::Tab5AudioTonePhase::Ready);
  CHECK_EQ(screen.tones_played(), 0u);
  CHECK_EQ(screen.touch(true, 360, 200, 1000), hgp::Tab5AudioAction::PlayTone);
  CHECK_EQ(screen.touch(false, 360, 200, 1010), hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.tones_played(), 1u);
  // A tap inside the cooldown is refused, not queued behind the first tone.
  CHECK_EQ(screen.touch(true, 360, 210, 1100), hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.touch(false, 360, 210, 1110), hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.touch(true, 360, 220, 1950), hgp::Tab5AudioAction::None);  // 950 ms
  CHECK_EQ(screen.touch(false, 360, 220, 1960), hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.tones_played(), 1u);
  // One tone again after the cooldown, and the counter follows it.
  CHECK_EQ(screen.touch(true, 360, 200, 2000), hgp::Tab5AudioAction::PlayTone);
  CHECK_EQ(screen.touch(false, 360, 200, 2010), hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.tones_played(), 2u);
  screen.tone_done(hgp::Tab5SpeakerState::SafeOff, 2300);
  CHECK(screen.tick(3500));
  CHECK_EQ(screen.model().headline, std::string("TONE: off and verified"));
  CHECK(screen.model().detail.find("TONES PLAYED: 2   LAST: verified") != std::string::npos);

  // A tone whose off request could not be verified never claims success.
  CHECK_EQ(screen.touch(true, 360, 200, 3600), hgp::Tab5AudioAction::PlayTone);
  CHECK_EQ(screen.touch(false, 360, 200, 3610), hgp::Tab5AudioAction::None);
  screen.tone_done(hgp::Tab5SpeakerState::Unknown, 3900);
  CHECK(screen.tick(5100));
  CHECK_EQ(screen.model().headline, std::string("TONE: unknown"));
  CHECK(screen.model().detail.find("TONES PLAYED: 3   LAST: UNKNOWN") != std::string::npos);
  CHECK(screen.model().detail.find("off and verified") == std::string::npos);
  CHECK(screen.model().detail.find("do not assume") != std::string::npos);
  CHECK_EQ(int(screen.model().caption_lines), 4);
  // The unknown stays on screen after the held state returns to ready.
  CHECK(screen.tick(6600));
  CHECK_EQ(screen.model().headline, std::string("TONE: ready"));
  CHECK(screen.model().detail.find("LAST: UNKNOWN") != std::string::npos);
  CHECK(screen.model().detail.find("do not assume") != std::string::npos);
  CHECK_EQ(int(screen.model().caption_lines), 4);
  // A later verified tone is the honest end state again.
  CHECK_EQ(screen.touch(true, 360, 200, 7000), hgp::Tab5AudioAction::PlayTone);
  CHECK_EQ(screen.touch(false, 360, 200, 7010), hgp::Tab5AudioAction::None);
  screen.tone_done(hgp::Tab5SpeakerState::SafeOff, 7300);
  CHECK(screen.tick(8500));
  CHECK_EQ(screen.model().headline, std::string("TONE: off and verified"));
  CHECK(screen.model().detail.find("TONES PLAYED: 4   LAST: verified") != std::string::npos);
  CHECK(screen.model().detail.find("do not assume") == std::string::npos);
  CHECK_EQ(int(screen.model().caption_lines), 3);

  // No tone while the microphone window owns the ADC, and the counter is unchanged.
  CHECK_EQ(screen.touch(true, 360, 1000, 10000), hgp::Tab5AudioAction::MeasureMic);
  CHECK_EQ(screen.touch(false, 360, 1000, 10010), hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.touch(true, 360, 200, 11000), hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.touch(false, 360, 200, 11010), hgp::Tab5AudioAction::None);
  CHECK_EQ(screen.tones_played(), 4u);
  screen.mic_done(hgp::Tab5MicStats{});
}

TEST("Tab5 audio UI: each tone drives the confirmed HIGH enable and then verifies off") {
  // Volume 75, one tone per call, with the mute/enable order and the final off
  // readback observable through an ordered event string.
  struct ToneFake {
    std::array<uint8_t, 256> regs{};
    std::string events;  // O open, u unmute, t tone, m mute, C close, h/l enable bit
    std::vector<bool> high_at_tone;
    unsigned opens = 0, closes = 0;
    bool toned = false;
    bool lie_after_tone = false;  // the off readback disagrees after the tone
    uint64_t time = 0;
    uint64_t now_ms() const { return time; }
    bool read(uint8_t reg, uint8_t& value, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 50);
      time += 1;
      value = regs[reg];
      if (lie_after_tone && toned) value = static_cast<uint8_t>(value ^ 0x02);
      return true;
    }
    bool write(uint8_t reg, uint8_t value, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 50);
      time += 1;
      // Masked read-modify-write: no unrelated bit of the expander ever changes.
      CHECK_EQ(int(value & ~0x02), int(regs[reg] & ~0x02));
      if (reg == hgp::kTab5SpeakerConfigReg) events.push_back((value & 0x02) ? 'h' : 'l');
      regs[reg] = value;
      return true;
    }
    bool open_dac(uint8_t volume, bool muted, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 500);
      CHECK_EQ(int(volume), int(hgp::kTab5AudioVolume));
      CHECK(muted);  // nothing is audible before the tone's own unmute
      events.push_back('O');
      ++opens;
      time += 5;
      return true;
    }
    bool set_mute(bool muted, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 50);
      events.push_back(muted ? 'm' : 'u');
      time += 1;
      return true;
    }
    bool tone(const int16_t* pcm, size_t frames, uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 1000);
      CHECK_EQ(frames, hgp::kTab5SpeakerToneFrames);
      for (size_t i = 0; i < frames; ++i)
        CHECK(pcm[i] == (i % 24 < 12 ? hgp::kTab5AudioTonePeak : -hgp::kTab5AudioTonePeak));
      high_at_tone.push_back((regs[hgp::kTab5SpeakerConfigReg] & hgp::kTab5SpeakerMask) != 0);
      events.push_back('t');
      toned = true;
      time += 250;
      return true;
    }
    bool close_dac(uint32_t timeout) {
      CHECK(timeout > 0 && timeout <= 500);
      events.push_back('C');
      ++closes;
      time += 1;
      return true;
    }
  };

  ToneFake io;
  io.regs[0x05] = 0xA7;  // pin 1 high: the off request must clear exactly that bit
  io.regs[0x07] = 0x0F;
  io.regs[0x03] = 0x0F;
  hgp::Tab5AudioScreen screen(720, 1280);
  auto play = [&](uint64_t now) {
    hgp::Tab5SpeakerDiagnostic<ToneFake> diagnostic(io);
    const bool prepared = diagnostic.prepare(hgp::kTab5AudioVolume);
    const bool played = prepared && diagnostic.play_once(true, hgp::kTab5AudioTonePeak);
    screen.tone_done(diagnostic.state(), now);
    return played;
  };
  CHECK_EQ(screen.touch(true, 360, 200, 1000), hgp::Tab5AudioAction::PlayTone);
  CHECK_EQ(screen.touch(false, 360, 200, 1010), hgp::Tab5AudioAction::None);
  CHECK(play(1000));
  // One per tap: off, open muted, enable HIGH, unmute, tone, mute, off, close.
  CHECK_EQ(io.events, std::string("lOhutmlC"));
  CHECK_EQ(io.high_at_tone.size(), size_t{1});
  CHECK(io.high_at_tone[0]);  // the confirmed HIGH enable, never the inverted level
  CHECK_EQ(io.opens, 1u);
  CHECK_EQ(io.closes, 1u);
  CHECK_EQ(int(io.regs[0x05]), 0xA5);  // reviewed off state, unrelated bits kept
  CHECK_EQ(int(io.regs[0x05] & 0x02), 0);
  CHECK_EQ(int(io.regs[0x07] & 0x02), 0);
  CHECK_EQ(int(io.regs[0x03] & 0x02), 0x02);
  CHECK_EQ(int(io.regs[0x07] & ~0x02), 0x0F & ~0x02);
  CHECK_EQ(int(io.regs[0x03] & ~0x02), 0x0F & ~0x02);
  CHECK(screen.tick(2500));  // the play line was held, then the result
  CHECK_EQ(screen.model().headline, std::string("TONE: off and verified"));
  // The next tap is a second tone, not a spent probe: the DAC is reopened,
  // driven HIGH again and closed again.
  CHECK_EQ(screen.touch(true, 360, 200, 2000), hgp::Tab5AudioAction::PlayTone);
  CHECK_EQ(screen.touch(false, 360, 200, 2010), hgp::Tab5AudioAction::None);
  CHECK(play(2000));
  CHECK_EQ(io.events, std::string("lOhutmlClOhutmlC"));
  CHECK_EQ(io.high_at_tone.size(), size_t{2});
  CHECK(io.high_at_tone[1]);
  CHECK_EQ(io.opens, 2u);
  CHECK_EQ(io.closes, 2u);
  CHECK_EQ(int(io.regs[0x05]), 0xA5);

  // The off request that the expander does not confirm leaves Unknown, and the
  // screen never shows it as a verified off.
  ToneFake bad;
  bad.regs[0x05] = 0xA7;
  bad.regs[0x07] = 0x0F;
  bad.regs[0x03] = 0x0F;
  bad.lie_after_tone = true;
  hgp::Tab5AudioScreen failed(720, 1280);
  CHECK_EQ(failed.touch(true, 360, 200, 1000), hgp::Tab5AudioAction::PlayTone);
  hgp::Tab5SpeakerDiagnostic<ToneFake> diagnostic(bad);
  const bool prepared = diagnostic.prepare(hgp::kTab5AudioVolume);
  CHECK(!(prepared && diagnostic.play_once(true, hgp::kTab5AudioTonePeak)));
  CHECK(diagnostic.state() == hgp::Tab5SpeakerState::Unknown);
  CHECK_EQ(bad.high_at_tone.size(), size_t{1});  // the tone did play
  CHECK(bad.high_at_tone[0]);
  failed.tone_done(diagnostic.state(), 1300);
  CHECK(failed.tick(2500));
  CHECK_EQ(failed.model().headline, std::string("TONE: unknown"));
  CHECK(failed.model().detail.find("LAST: UNKNOWN") != std::string::npos);
  CHECK(failed.model().detail.find("off and verified") == std::string::npos);
  CHECK(failed.model().detail.find("do not assume") != std::string::npos);
}

TEST("Tab5: profile enables the radio and does not inherit GPIO45/46 LDO") {
  const auto b = hgp::tab5_config("m5stack-tab5");
  // The radio is on: the C6 SDIO map lives in this board's sdkconfig defaults
  // and main.cpp starts ESP-Hosted in the normal startup path only, never in
  // the pre-main constructor or the display/audio diagnostic modes.
  CHECK(b.radio_enabled);
  CHECK(!b.lcd.dsi.vdd_io_ldo4);
  // The codec profile is carried now (the measured ES8388/ES7210 path); the
  // chip's own I2S mic/speaker and the P4 codec-less speaker stay unused.
  CHECK(b.codec.enabled && !b.mic.enabled && !b.speaker.enabled && !b.p4_speaker.enabled);
  CHECK(b.lcd.enabled && b.touch.enabled);
  CHECK_EQ(b.lcd.width, 720);
  CHECK_EQ(b.lcd.height, 1280);
  CHECK_EQ(b.i2c.sda, 31);
  CHECK_EQ(b.i2c.scl, 32);
  CHECK_EQ(b.touch.addr, 0x14);
  CHECK_EQ(b.touch.int_gpio, -1);
  CHECK(!b.lcd.swap_xy && !b.lcd.mirror_x && !b.lcd.mirror_y);
  CHECK(!b.touch.swap_xy && !b.touch.mirror_x && !b.touch.mirror_y);
  // Existing profiles that do not opt out retain both startup policies.
  const hgp::BoardConfig legacy{};
  CHECK(legacy.radio_enabled);
  CHECK(legacy.lcd.dsi.vdd_io_ldo4);
}

// The hosted transport is wiring, not code: check_config.py already fails a
// build whose defaults never reached the generated sdkconfig, so what is left
// to pin here is which map those defaults carry -- this unit's historical
// factory boot log (CLK 12, CMD 13, D0-D3 11/10/9/8, reset 15) and never the
// ESP-Hosted P4-dev-board default (CMD 19, CLK 18, D0-D3 14-17, reset 54).
TEST("Tab5: hosted SDIO defaults carry the factory C6 map") {
  const std::string source = __FILE__;
  const std::string here = source.substr(0, source.find_last_of("/\\") + 1);
  // Make/Ninja compile with an absolute path; a generator that emits a
  // relative one still resolves against the build tree ctest runs in.
  const std::string candidates[] = {
      here + "../esp32/boards/m5stack-tab5/sdkconfig.defaults",
      "../../firmware/esp32/boards/m5stack-tab5/sdkconfig.defaults",
  };
  std::ifstream file;
  for (const auto& path : candidates) {
    file.open(path);
    if (file) break;
    file.clear();
  }
  CHECK(file.is_open());
  if (!file.is_open()) return;
  auto trim = [](std::string line) {
    const auto first = line.find_first_not_of(" \t");
    line = first == std::string::npos ? std::string() : line.substr(first);
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) line.pop_back();
    return line;
  };
  std::map<std::string, std::string> defaults;
  std::string raw;
  while (std::getline(file, raw)) {
    const std::string line = trim(raw);
    if (line.rfind("CONFIG_", 0) == 0) {
      const auto eq = line.find('=');
      if (eq != std::string::npos) defaults[line.substr(7, eq - 7)] = line.substr(eq + 1);
    } else if (line.rfind("# CONFIG_", 0) == 0 && line.size() > 20 &&
               line.compare(line.size() - 11, 11, " is not set") == 0) {
      defaults[line.substr(9, line.size() - 9 - 11)] = "n";
    }
  }
  const auto value = [&](const std::string& symbol) {
    const auto it = defaults.find(symbol);
    return it == defaults.end() ? std::string("<absent>") : it->second;
  };
  // Slot 1 is the P4's C6 slot (slot 0 is the microSD card the BSP owns).
  CHECK_EQ(value("ESP_HOSTED_SDIO_HOST_INTERFACE"), std::string("y"));
  CHECK_EQ(value("ESP_HOSTED_SDIO_SLOT_1"), std::string("y"));
  CHECK_EQ(value("ESP_HOSTED_SDIO_SLOT_0"), std::string("n"));
  CHECK_EQ(value("ESP_HOSTED_SDIO_4_BIT_BUS"), std::string("y"));
  CHECK_EQ(value("ESP_HOSTED_SDIO_BUS_WIDTH"), std::string("4"));
  CHECK_EQ(value("ESP_HOSTED_SDIO_CLOCK_FREQ_KHZ"), std::string("40000"));
  // The pins the factory firmware printed: CLK 12, CMD 13, D0-D3 11/10/9/8.
  CHECK_EQ(value("ESP_HOSTED_PRIV_SDIO_PIN_CLK_SLOT_1"), std::string("12"));
  CHECK_EQ(value("ESP_HOSTED_PRIV_SDIO_PIN_CMD_SLOT_1"), std::string("13"));
  CHECK_EQ(value("ESP_HOSTED_PRIV_SDIO_PIN_D0_SLOT_1"), std::string("11"));
  CHECK_EQ(value("ESP_HOSTED_PRIV_SDIO_PIN_D1_4BIT_BUS_SLOT_1"), std::string("10"));
  CHECK_EQ(value("ESP_HOSTED_PRIV_SDIO_PIN_D2_4BIT_BUS_SLOT_1"), std::string("9"));
  CHECK_EQ(value("ESP_HOSTED_PRIV_SDIO_PIN_D3_4BIT_BUS_SLOT_1"), std::string("8"));
  // Slave reset on the pin the factory log drove, active high: the line rests
  // HIGH and a LOW pulse resets, leaving the C6's EN running afterwards.
  CHECK_EQ(value("ESP_HOSTED_SDIO_GPIO_RESET_SLAVE"), std::string("15"));
  CHECK_EQ(value("ESP_HOSTED_SDIO_RESET_ACTIVE_HIGH"), std::string("y"));
  CHECK_EQ(value("ESP_HOSTED_SDIO_RESET_ACTIVE_LOW"), std::string("n"));
  CHECK_EQ(value("ESP_HOSTED_IDF_SLAVE_TARGET"), std::string("\"esp32c6\""));
  // Neither the pre-main mempool allocation nor the host restart on a
  // transport failure: a wrong map must report, not reboot the unit.
  CHECK_EQ(value("ESP_HOSTED_USE_MEMPOOL"), std::string("n"));
  CHECK_EQ(value("ESP_HOSTED_TRANSPORT_RESTART_ON_FAILURE"), std::string("n"));
}

TEST("Tab5: profile carries the measured ES8388 speaker and ES7210 microphone") {
  const auto b = hgp::tab5_config("m5stack-tab5");
  CHECK(b.codec.enabled);
  CHECK(b.codec.speaker == hgp::SpeakerCodec::Es8388);
  CHECK(b.codec.mic == hgp::MicCodec::Es7210);
  // One duplex I2S_NUM_1 bus: MCLK 30, BCLK 27, WS 29, DOUT 26, DIN 28.
  CHECK_EQ(b.codec.mclk, 30);
  CHECK_EQ(b.codec.bclk, 27);
  CHECK_EQ(b.codec.ws, 29);
  CHECK_EQ(b.codec.dout, 26);
  CHECK_EQ(b.codec.din, 28);
  // The amplifier is the 0x43 expander's pin 1, never a GPIO, so no codec
  // driver may drive one; the format stays the 24 kHz mono 16-bit one every
  // SDK codec board runs.
  CHECK_EQ(b.codec.pa, -1);
  CHECK(!b.codec.speaker_pa);
  CHECK(!b.codec.stereo32);
  CHECK(!b.codec.rmnm_mics);
  // MIC1|MIC2 and a gain inside what the working microphone probe opened.
  CHECK_EQ(b.codec.es7210_mics, 0x03);
  CHECK(b.codec.mic_gain_db > 0.0f && b.codec.mic_gain_db <= 24.0f);
  // Measured here: -40 dBFS at volume 40 was silent, -12 dBFS at 75 was heard.
  CHECK(b.codec.volume >= 50 && b.codec.volume <= 100);
  CHECK(b.codec.amp_supply_v > 0.0f);
  // The defaults other boards inherit are unchanged by this profile.
  const hgp::CodecAudioConfig defaults{};
  CHECK(!defaults.enabled);
  CHECK(defaults.speaker == hgp::SpeakerCodec::Es8311);
  CHECK_EQ(defaults.volume, 70);
  CHECK_EQ(defaults.mic_gain_db, 24.0f);
  CHECK_EQ(b.codec.mic_gain_db, defaults.mic_gain_db);
}

namespace {
// The Tab5 amplifier's IO contract, on the numbered registers the reviewed
// off sequence uses: read() and write() reach the 0x43 expander only, and
// mute() silences the codec a failed request could not prove off.
// `fail_at` fails the Nth operation of any kind; `apply_failed_write` models a
// write that reached the chip although the bus reported an error;
// `corrupt_readback` makes the readback of the last written register disagree
// with the value written.
struct Tab5AmplifierFake {
  std::array<uint8_t, 256> regs{};
  std::vector<std::pair<uint8_t, uint8_t>> writes;  // (register, value), in order
  unsigned reads = 0, mutes = 0, operations = 0, fail_at = 0;
  bool apply_failed_write = false, corrupt_readback = false;
  uint8_t pending_readback = 0;  // register whose value was just written
  bool read(uint8_t reg, uint8_t& value, uint32_t timeout) {
    CHECK(timeout > 0);
    ++reads;
    value = regs[reg];
    if (corrupt_readback && pending_readback == reg) value ^= 0x02;
    pending_readback = 0;
    return ++operations != fail_at;
  }
  bool write(uint8_t reg, uint8_t value, uint32_t timeout) {
    CHECK(timeout > 0);
    const bool ok = ++operations != fail_at;
    if (ok || apply_failed_write) {
      regs[reg] = value;
      writes.emplace_back(reg, value);
      pending_readback = reg;
    }
    return ok;
  }
  void mute() { ++mutes; }
};
}  // namespace

TEST("Tab5: amplifier drives only expander 0x43 pin 1, HIGH enabled") {
  for (unsigned seed : {0u, 0xffu, 0x55u, 0xaau}) {
    Tab5AmplifierFake io;
    io.regs.fill(static_cast<uint8_t>(seed));
    hgp::Tab5Amplifier<Tab5AmplifierFake> amp(io);
    CHECK(!amp.failed());
    // The reviewed off state, in the order tab5_prepare_speaker_off uses:
    // 0x05 the control bit clear, 0x07 an output, 0x03 driven (off level).
    CHECK(amp.prepare_off());
    CHECK_EQ(io.writes.size(), size_t{3});
    CHECK_EQ(io.writes[0].first, uint8_t{0x05});
    CHECK_EQ(io.writes[1].first, uint8_t{0x07});
    CHECK_EQ(io.writes[2].first, uint8_t{0x03});
    for (const auto& [reg, value] : io.writes) {
      CHECK_EQ(value & ~0x02, seed & ~0x02u);  // every other expander pin preserved
      CHECK(reg != 0x01);                      // never a chip reset
    }
    CHECK_EQ(io.regs[0x05] & 0x02, 0);
    CHECK_EQ(io.regs[0x07] & 0x02, 0);
    CHECK_EQ(io.regs[0x03] & 0x02, 0x02);
    // Every write is followed by a readback, and nothing failed.
    CHECK_EQ(io.operations, 9u);
    CHECK_EQ(io.reads, 2 * io.writes.size());
    CHECK(!amp.failed());
    CHECK_EQ(io.mutes, 0u);
    // Enable: the measured convention, HIGH = enabled, as one masked bit of
    // 0x05 that is read back.
    CHECK(amp.set(true));
    CHECK_EQ(io.writes.size(), size_t{4});
    CHECK_EQ(io.writes.back().first, uint8_t{0x05});
    CHECK_EQ(io.writes.back().second, static_cast<uint8_t>((seed & ~0x02u) | 0x02));
    CHECK_EQ(io.regs[0x05] & 0x02, 0x02);
    CHECK_EQ(io.regs[0x07], static_cast<uint8_t>(seed & ~0x02u));
    CHECK_EQ(io.regs[0x03] & ~0x02, seed & ~0x02u);
    CHECK_EQ(io.operations, 12u);
    CHECK_EQ(io.reads, 2 * io.writes.size());
    CHECK(!amp.failed());
    // Off again: one masked bit, read back, with the prepared direction and
    // level left alone, so a tap can enable it again.
    CHECK(amp.set(false));
    CHECK_EQ(io.regs[0x05] & 0x02, 0);
    CHECK_EQ(io.regs[0x07] & 0x02, 0);
    CHECK_EQ(io.regs[0x03] & 0x02, 0x02);
    CHECK(amp.set(true));
    CHECK_EQ(io.regs[0x05] & 0x02, 0x02);
    CHECK_EQ(io.reads, 2 * io.writes.size());
    CHECK(!amp.failed());
    CHECK_EQ(io.mutes, 0u);
  }
}

TEST("Tab5: amplifier latches a failed write, mutes once and refuses to enable") {
  for (bool applies : {false, true}) for (unsigned fail_at = 1; fail_at <= 12; ++fail_at) {
    Tab5AmplifierFake io;
    io.regs.fill(0xff);
    io.operations = 0;
    io.fail_at = fail_at;
    io.apply_failed_write = applies;
    hgp::Tab5Amplifier<Tab5AmplifierFake> amp(io);
    const bool prepared = amp.prepare_off();
    if (prepared) amp.set(true);
    // Whichever operation failed, the state is unknown from then on.
    CHECK(amp.failed());
    CHECK_EQ(io.mutes, 1u);  // the codec is muted exactly once
    // A failed request may have left the amplifier on: no later request may
    // enable it, and none may claim it is off.
    const unsigned operations = io.operations;
    CHECK(!amp.set(true));
    CHECK(!amp.set(false));
    CHECK(!amp.prepare_off());
    CHECK_EQ(io.operations, operations);  // no I2C after the latch
    CHECK_EQ(io.mutes, 1u);
  }
  // A write the bus acknowledged but the chip did not keep is a failure too.
  Tab5AmplifierFake io;
  io.regs.fill(0xa7);
  io.corrupt_readback = true;
  hgp::Tab5Amplifier<Tab5AmplifierFake> amp(io);
  CHECK(!amp.prepare_off());
  CHECK(amp.failed());
  CHECK_EQ(io.mutes, 1u);
  const unsigned operations = io.operations;
  CHECK(!amp.set(true));
  CHECK_EQ(io.operations, operations);
}

TEST("Tab5: a failed amplifier request is reported as unknown, never as off") {
  const std::string line = hgp::kTab5PaFailureLog;
  CHECK(line.find("Tab5 amplifier") != std::string::npos);
  CHECK(line.find("0x43") != std::string::npos);
  CHECK(line.find("pin 1") != std::string::npos);
  CHECK(line.find("muted") != std::string::npos);
  CHECK(line.find("unknown") != std::string::npos);
  CHECK(line.find("not off") != std::string::npos);
  CHECK(line.find("reboot") != std::string::npos);
  CHECK(line.find("amplifier is off") == std::string::npos);
}

TEST("Tab5: C6 power gate drives only pin 0 of the second expander") {
  for (unsigned initial : {0u, 0xffu, 0x55u, 0xaau}) {
    std::array<uint8_t, 256> regs;
    regs.fill(static_cast<uint8_t>(initial));
    std::vector<uint8_t> order;
    CHECK(hgp::tab5_enable_wifi_power(
        [&](uint8_t reg, uint8_t& value) { value = regs[reg]; return true; },
        [&](uint8_t reg, uint8_t value) {
          CHECK_EQ(value & ~0x01u, regs[reg] & ~0x01u);
          regs[reg] = value;
          order.push_back(reg);
          return true;
        }));
    // Latch high while still an input, then push-pull, then output: no low glitch.
    CHECK(order == std::vector<uint8_t>({0x05, 0x07, 0x03}));
    CHECK_EQ(regs[0x05] & 0x01, 0x01);
    CHECK_EQ(regs[0x07] & 0x01, 0x00);
    CHECK_EQ(regs[0x03] & 0x01, 0x01);
  }
  int operations = 0;
  std::array<uint8_t, 256> regs;
  regs.fill(0);
  CHECK(!hgp::tab5_enable_wifi_power(
      [&](uint8_t reg, uint8_t& value) { value = regs[reg]; return operations++ != 2; },
      [&](uint8_t reg, uint8_t value) { regs[reg] = value; return operations++ != 2; }));
}

TEST("Tab5: initialize speaker amplifier off without changing other expander pins") {
  for (unsigned initial : {0u, 0xffu, 0x55u, 0xaau}) for (unsigned bits = 0; bits < 8; ++bits) {
    std::array<uint8_t, 256> regs;
    regs.fill(static_cast<uint8_t>(initial));
    regs[0x05] = static_cast<uint8_t>((regs[0x05] & ~0x02u) | ((bits & 1) ? 0x02 : 0));
    regs[0x07] = static_cast<uint8_t>((regs[0x07] & ~0x02u) | ((bits & 2) ? 0x02 : 0));
    regs[0x03] = static_cast<uint8_t>((regs[0x03] & ~0x02u) | ((bits & 4) ? 0x02 : 0));
    std::vector<uint8_t> writes;
    CHECK(hgp::tab5_prepare_speaker_off(
        [&](uint8_t reg, uint8_t& value) { value = regs[reg]; return true; },
        [&](uint8_t reg, uint8_t value) {
          CHECK_EQ(value & ~0x02, regs[reg] & ~0x02);
          regs[reg] = value;
          writes.push_back(reg);
          return true;
        }));
    CHECK(writes == std::vector<uint8_t>({0x05, 0x07, 0x03}));
    CHECK_EQ(regs[0x05] & 0x02, 0);
    CHECK_EQ(regs[0x07] & 0x02, 0);
    CHECK_EQ(regs[0x03] & 0x02, 0x02);
  }
}

TEST("Tab5: abort speaker setup on the first failed I2C operation") {
  for (int failed = 0; failed < 6; ++failed) for (bool applies : {false, true}) {
    int operations = 0;
    std::array<uint8_t, 256> regs;
    regs.fill(0xff);
    CHECK(!hgp::tab5_prepare_speaker_off(
        [&](uint8_t reg, uint8_t& value) { value = regs[reg]; return operations++ != failed; },
        [&](uint8_t reg, uint8_t value) {
          CHECK_EQ(value & ~0x02, regs[reg] & ~0x02);
          const bool ok = operations++ != failed;
          if (ok || applies) regs[reg] = value;
          return ok;
        }));
    CHECK_EQ(operations, failed + 1);
    CHECK_EQ(regs[0x03] & ~0x02, 0xfd);
    CHECK_EQ(regs[0x05] & ~0x02, 0xfd);
    CHECK_EQ(regs[0x07] & ~0x02, 0xfd);
  }
}

TEST("Tab5: expander enables only LCD and touch, preserving unrelated bits") {
  for (unsigned seed : {0u, 0xffu, 0x55u, 0xaau}) {
    std::array<uint8_t, 256> regs;
    regs.fill(seed);
    unsigned writes = 0;
    CHECK(hgp::tab5_enable_display_touch(
        [&](uint8_t reg, uint8_t& value) { value = regs[reg]; return true; },
        [&](uint8_t reg, uint8_t value) {
          CHECK(reg != 0x01);  // never reset the entire expander
          CHECK_EQ(value & ~0x30, regs[reg] & ~0x30);
          regs[reg] = value;
          ++writes;
          return true;
        }));
    CHECK(writes > 0);
    CHECK_EQ(regs[0x03] & 0x30, 0x20);  // LCD input, touch output
    CHECK_EQ(regs[0x05] & 0x20, 0x20);  // touch enabled
    CHECK_EQ(regs[0x07] & 0x30, 0x10);  // LCD high-Z, touch push-pull
    CHECK_EQ(regs[0x0b] & 0x10, 0x10);  // LCD pull enabled
    CHECK_EQ(regs[0x0d] & 0x10, 0x10);  // LCD pull up
  }
}

TEST("Tab5: expander setup stops on each failed read or write") {
  for (int fail = 0; fail < 10; ++fail) {
    int operations = 0;
    CHECK(!hgp::tab5_enable_display_touch(
        [&](uint8_t, uint8_t& value) { value = 0; return operations++ != fail; },
        [&](uint8_t, uint8_t) { return operations++ != fail; }));
    CHECK(operations >= fail + 1);  // rollback may perform additional I2C operations
  }
}

TEST("Tab5: partial expander failures restore modified register bits when I2C recovers") {
  for (int fail = 0; fail < 10; ++fail) {
    std::array<uint8_t, 256> regs{};
    regs.fill(0x55);
    const auto original = regs;
    int operations = 0;
    CHECK(!hgp::tab5_enable_display_touch(
        [&](uint8_t reg, uint8_t& value) {
          if (operations++ == fail) return false;
          value = regs[reg];
          return true;
        },
        [&](uint8_t reg, uint8_t value) {
          if (operations++ == fail) return false;
          regs[reg] = value;
          return true;
        }));
    for (uint8_t reg : {0x03, 0x05, 0x07, 0x0b, 0x0d}) CHECK_EQ(regs[reg], original[reg]);
  }
}

TEST("Tab5: a failed expander write may still have changed the device") {
  for (int failed_write = 0; failed_write < 5; ++failed_write) {
    std::array<uint8_t, 256> regs{};
    regs.fill(0xaa);
    const auto original = regs;
    int writes = 0;
    CHECK(!hgp::tab5_enable_display_touch(
        [&](uint8_t reg, uint8_t& value) { value = regs[reg]; return true; },
        [&](uint8_t reg, uint8_t value) {
          regs[reg] = value;  // Device accepted the transaction before the error.
          return writes++ != failed_write;
        }));
    for (uint8_t reg : {0x03, 0x05, 0x07, 0x0b, 0x0d}) CHECK_EQ(regs[reg], original[reg]);
  }
}

TEST("Tab5: rollback continues after an I2C restore failure") {
  for (bool fail_read : {false, true}) {
    std::array<uint8_t, 256> regs{};
    regs.fill(0x55);
    bool restoring = false;
    bool failed = false;
    unsigned restore_writes = 0;
    CHECK(!hgp::tab5_enable_display_touch(
        [&](uint8_t reg, uint8_t& value) {
          if (restoring && fail_read && !failed) { failed = true; return false; }
          value = regs[reg];
          return true;
        },
        [&](uint8_t reg, uint8_t value) {
          if (restoring) {
            ++restore_writes;
            if (!fail_read && !failed) { failed = true; return false; }
          }
          regs[reg] = value;
          return true;
        },
        [&] { restoring = true; return false; }));
    CHECK(failed);
    CHECK_EQ(restore_writes, fail_read ? 4u : 5u);
    CHECK_EQ(regs[0x05], 0x55);  // earliest change is still restored
  }
}

TEST("Tab5: rejected touch probe restores expander state") {
  std::array<uint8_t, 256> regs{};
  regs.fill(0x55);
  const auto original = regs;
  CHECK(!hgp::tab5_enable_display_touch(
      [&](uint8_t reg, uint8_t& value) { value = regs[reg]; return true; },
      [&](uint8_t reg, uint8_t value) { regs[reg] = value; return true; },
      [] { return false; }));
  for (uint8_t reg : {0x03, 0x05, 0x07, 0x0b, 0x0d}) CHECK_EQ(regs[reg], original[reg]);
}

TEST("battery estimate: the shared single-cell map clamps endpoints and floors intermediate levels") {
  struct Sample { uint16_t mv; uint8_t percent; };
  for (const Sample& sample : {Sample{0, 0}, Sample{2000, 0}, Sample{3299, 0}, Sample{3300, 0},
                               Sample{3308, 0}, Sample{3309, 1}, Sample{3549, 27}, Sample{3750, 50},
                               Sample{4000, 77}, Sample{4199, 99}, Sample{4200, 100},
                               Sample{4500, 100}, Sample{65535, 100}}) {
    CHECK_EQ(hg::estimate_liion_percent(sample.mv), sample.percent);
  }
}

TEST("STC8 battery: reads register bytes and maps only the Gadget power fields") {
  const std::array<uint8_t, 12> raw = {0xeb, 0x0b, 0, 0, 0x90, 0x10, 0, 0, 0x64, 0x02, 0x02, 0xaa};
  std::vector<uint8_t> registers;
  hg::Stc8Battery power([&](uint8_t reg, uint8_t& value) {
    registers.push_back(reg);
    value = raw[reg];
    return true;
  });

  const auto status = power.read();

  CHECK(status.has_value());
  CHECK(registers == std::vector<uint8_t>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}));
  CHECK(status->battery_mv == uint16_t(4240));
  CHECK(status->battery_percent == uint8_t(100));
  CHECK(status->charging == false);
  CHECK(!power.can_power_off());
  CHECK(!power.power_off());
}

TEST("STC8 battery: charging state maps to the generic charging flag") {
  const std::array<uint8_t, 12> raw = {0xeb, 0x0b, 0, 0, 0x88, 0x10, 0, 0, 75, 1, 1, 0};
  hg::Stc8Battery power([&](uint8_t reg, uint8_t& value) { value = raw[reg]; return true; });

  const auto status = power.read();

  CHECK(status.has_value());
  CHECK(status->charging == true);
}

TEST("STC8 battery: error state is preserved without claiming not charging") {
  const std::array<uint8_t, 12> raw = {0xeb, 0x0b, 0, 0, 0x90, 0x10, 0, 0, 100, 4, 4, 0};
  hg::Stc8Battery power([&](uint8_t reg, uint8_t& value) { value = raw[reg]; return true; });

  const auto status = power.read();

  CHECK(status.has_value());
  CHECK(!status->charging.has_value());
}

TEST("STC8 battery: a register read failure discards the sample and stops the sequence") {
  std::vector<uint8_t> registers;
  hg::Stc8Battery power([&](uint8_t reg, uint8_t& value) {
    registers.push_back(reg);
    if (reg == 5) return false;
    value = 0;
    return true;
  });

  CHECK(!power.read());
  CHECK(registers == std::vector<uint8_t>({0, 1, 2, 3, 4, 5}));
}

TEST("STC8 battery: invalid values are omitted rather than reported as real telemetry") {
  const std::array<uint8_t, 12> raw = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 101, 5, 5, 0xaa};
  hg::Stc8Battery power([&](uint8_t reg, uint8_t& value) { value = raw[reg]; return true; });

  const auto status = power.read();

  CHECK(status.has_value());
  CHECK(!status->battery_mv);
  CHECK(!status->battery_percent);
  CHECK(!status->charging);
  CHECK(!status->battery_present);
  CHECK(!status->external_power);
}

TEST("battery estimate: the resting-voltage curve clamps endpoints, interpolates and floors") {
  struct Sample { uint16_t mv; uint8_t percent; };
  for (const Sample& sample : {Sample{0, 0}, Sample{3300, 0}, Sample{3455, 2}, Sample{3610, 5}, Sample{3750, 25},
                               Sample{3840, 50}, Sample{3867, 59}, Sample{3909, 64}, Sample{4110, 90},
                               Sample{4199, 99}, Sample{4200, 100}, Sample{65535, 100}}) {
    CHECK_EQ(hg::estimate_liion_curve_percent(sample.mv), sample.percent);
  }
  // Monotonic: a higher voltage never shows a lower level.
  for (uint16_t mv = 3300; mv < 4200; ++mv)
    CHECK(hg::estimate_liion_curve_percent(mv + 1) >= hg::estimate_liion_curve_percent(mv));
}

TEST("AXP2101: battery, USB and charging readings never alter power configuration") {
  std::array<uint8_t, 256> regs{};
  regs[0x00] = 0x28;
  regs[0x01] = 0x20;
  regs[0x18] = 0x0a;
  regs[0x30] = 0x03;
  regs[0x34] = 0xcf;  // high status bits must not become voltage bits
  regs[0x35] = 0xa0;  // 4000 mV
  regs[0xa4] = 81;
  int failed_reg = -1;
  std::vector<std::pair<uint8_t, uint8_t>> writes;
  hg::Axp2101 power(
      [&](uint8_t reg, uint8_t* out, size_t n) {
        if (reg == failed_reg) return false;
        for (size_t i = 0; i < n; ++i) out[i] = regs[reg + i];
        return true;
      },
      [&](uint8_t reg, uint8_t value) { writes.emplace_back(reg, value); return true; });
  auto p = power.read();
  CHECK(p.has_value());
  CHECK(p->battery_present == true);
  CHECK(p->external_power == true);
  CHECK(p->charging == true);
  CHECK(p->battery_mv == 4000);
  CHECK(p->battery_percent == 81);
  CHECK(writes.empty());

  regs[0xa4] = 255;
  CHECK_EQ(power.read()->battery_percent, uint8_t(77));  // junk gauge falls back to the voltage curve (4000 mV -> 77)
  regs[0xa4] = 0;
  p = power.read();
  CHECK_EQ(p->battery_percent, uint8_t(77));  // gauge stuck at 0: honest percent from voltage, no false "low battery"
  regs[0x34] = 0x0d;  // 3549 mV -> 27% on the 3300..4200 map
  regs[0x35] = 0xdd;
  CHECK_EQ(power.read()->battery_percent, uint8_t(27));
  regs[0x00] = 0x20;
  p = power.read();
  CHECK(p->battery_present == false);
  CHECK(p->charging == false);
  CHECK(!p->battery_mv);
  CHECK(!p->battery_percent);
  regs[0x00] = 0x08;
  regs[0x18] = regs[0x30] = 0;
  p = power.read();
  CHECK(!p->battery_mv);
  CHECK(!p->battery_percent);
  CHECK(writes.empty());
  failed_reg = 0x00;
  CHECK(!power.read());
  regs[0x00] = 0x28;
  regs[0x18] = 0x0a;
  regs[0x30] = 0x03;
  failed_reg = 0xa4;
  CHECK(!power.read());  // a gauge the bus can't read is unavailable, not estimated from voltage
  failed_reg = 0x10;
  CHECK(!power.power_off());
  CHECK(writes.empty());
  failed_reg = -1;
  regs[0x10] = 0x34;
  CHECK(power.power_off());
  CHECK_EQ(writes.size(), size_t(1));
  CHECK_EQ(writes[0].first, uint8_t(0x10));
  CHECK_EQ(writes[0].second, uint8_t(0x35));
}

TEST("AXP2101: audio supply enables ALDO1 at 3.3 V and preserves other rails") {
  std::array<uint8_t, 256> regs{};
  regs[0x90] = 0xa4;
  regs[0x92] = 0x60;
  regs[0x62] = 0x08;  // charge current must stay unchanged
  bool failed = false;
  hg::Axp2101 power(
      [&](uint8_t reg, uint8_t* out, size_t n) {
        if (failed) return false;
        for (size_t i = 0; i < n; ++i) out[i] = regs[reg + i];
        return true;
      },
      [&](uint8_t reg, uint8_t value) { regs[reg] = value; return true; });
  CHECK(power.enable_aldo1_3v3());
  CHECK_EQ(regs[0x90], uint8_t(0xa5));
  CHECK_EQ(regs[0x92], uint8_t(0x7c));
  CHECK_EQ(regs[0x62], uint8_t(0x08));
  failed = true;
  CHECK(!power.enable_aldo1_3v3());
}

TEST("CoreS3: peripheral power preserves charger and external output configuration") {
  std::array<uint8_t, 256> pmic{}, io{};
  pmic.fill(0x60);
  pmic[0x90] = 0xe4;
  io.fill(0xff);
  auto expected_pmic = pmic;
  auto expected_io = io;
  std::vector<uint32_t> delays;
  hg::CoreS3Control control(
      [&](uint8_t addr, uint8_t reg, uint8_t& value) {
        value = (addr == 0x34 ? pmic : io)[reg]; return true;
      },
      [&](uint8_t addr, uint8_t reg, uint8_t value) {
        (addr == 0x34 ? pmic : io)[reg] = value; return true;
      },
      [&](uint32_t ms) {
        delays.push_back(ms);
        if (ms == 10) { CHECK_EQ(io[0x02], 0xfa); CHECK_EQ(io[0x03], 0xfd); }
        if (ms == 300) { CHECK_EQ(io[0x02], 0xff); CHECK_EQ(io[0x03], 0xff); }
      });
  CHECK(control.begin());
  expected_pmic[0x92] = 0x6d;  // ALDO1: 1.8 V
  expected_pmic[0x93] = 0x7c;  // ALDO2: 3.3 V
  expected_pmic[0x90] = 0x67;  // enable audio rails, disable backlight
  expected_io[0x04] = 0xfa;
  expected_io[0x05] = 0x7d;
  CHECK(pmic == expected_pmic);
  CHECK(io == expected_io);
  CHECK(delays == std::vector<uint32_t>({10, 300}));
  CHECK(control.set_brightness(50));
  CHECK_EQ(pmic[0x99], 0x78);  // DLDO1: 2.9 V, upper bits retained
  CHECK_EQ(pmic[0x90], 0xe7);
  CHECK(control.set_brightness(255));
  CHECK_EQ(pmic[0x99], 0x7c);  // clamp to 3.3 V
  CHECK(control.set_brightness(0));
  CHECK_EQ(pmic[0x90], 0x67);
  CHECK_EQ(pmic[0x62], 0x60);  // charging current is untouched
  CHECK(io == expected_io);
}

TEST("CoreS3: failed power access stops initialization without releasing reset") {
  bool read_ok = false;
  int writes = 0, delays = 0;
  hg::CoreS3Control control(
      [&](uint8_t, uint8_t, uint8_t& value) { value = 0; return read_ok; },
      [&](uint8_t, uint8_t, uint8_t) { ++writes; return false; },
      [&](uint32_t) { ++delays; });
  CHECK(!control.begin());
  CHECK_EQ(writes, 0);
  read_ok = true;
  CHECK(!control.begin());
  CHECK_EQ(writes, 1);
  CHECK_EQ(delays, 0);
  CHECK(!control.set_brightness(100));
  CHECK_EQ(writes, 2);
}

namespace {
// The StickS3's M5PM1 as a register file.
struct FakePm1 {
  std::array<uint8_t, 256> regs{};
  bool fail = false;
  std::vector<uint8_t> written;
  hg::StickS3Control::Read read = [this](uint8_t reg, uint8_t* out, size_t n) {
    if (fail) return false;
    for (size_t i = 0; i < n; ++i) out[i] = regs[reg + i];
    return true;
  };
  hg::StickS3Control::Write write = [this](uint8_t reg, uint8_t value) {
    if (fail) return false;
    written.push_back(reg);
    regs[reg] = value;
    return true;
  };
};
}  // namespace

TEST("StickS3: the LCD rail comes on and the amplifier stays off; other PM1 pins are preserved") {
  FakePm1 fake;
  auto& pm1 = fake.regs;
  auto& written = fake.written;
  pm1.fill(0xff);
  pm1[0x09] = 0x01;  // I2C idle sleep left on by earlier firmware
  pm1[0x10] = 0x00;
  std::vector<uint32_t> delays;
  hg::StickS3Control control(fake.read, fake.write, [&](uint32_t ms) {
    delays.push_back(ms);
    CHECK_EQ(pm1[0x10], 0x0c);  // both outputs set before the wait, GPIO0 an input
  });
  CHECK(control.begin());
  CHECK_EQ(pm1[0x09], 0x00);
  CHECK_EQ(pm1[0x0a], 0x00);  // watchdog off
  CHECK_EQ(pm1[0x16], 0x0c);  // GPIO0/2/3 fields plain GPIO; GPIO1's field untouched
  CHECK_EQ(pm1[0x13], 0xf3);  // push-pull
  CHECK_EQ(pm1[0x11], 0xf7);  // LCD rail high, amplifier low
  CHECK(delays == std::vector<uint32_t>({100}));
  // Levels are set before the pins become outputs.
  const auto out = std::find(written.begin(), written.end(), uint8_t(0x11));
  const auto mode = std::find(written.begin(), written.end(), uint8_t(0x10));
  CHECK(out < mode);

  written.clear();
  CHECK(control.set_amplifier(true));
  CHECK_EQ(pm1[0x11], 0xff);
  CHECK(control.set_amplifier(true));  // no change, no write
  CHECK_EQ(written.size(), size_t(1));
  CHECK(control.set_amplifier(false));
  CHECK_EQ(pm1[0x11], 0xf7);
}

TEST("StickS3: a PM1 that does not answer stops initialization") {
  FakePm1 fake;
  fake.fail = true;
  int delays = 0;
  hg::StickS3Control control(fake.read, fake.write, [&](uint32_t) { ++delays; });
  CHECK(!control.begin());
  CHECK(fake.written.empty());
  CHECK_EQ(delays, 0);
  CHECK(!control.read());
  CHECK(!control.external_power());
}

TEST("StickS3: battery millivolts, USB and the active-low charge status are reported") {
  FakePm1 fake;
  hg::StickS3Control control(fake.read, fake.write, [](uint32_t) {});
  fake.regs[0x22] = 0x9c;  // 3996 mV, little-endian
  fake.regs[0x23] = 0x0f;
  fake.regs[0x04] = 0x05;  // USB and battery
  fake.regs[0x12] = 0x00;  // charge status low: charging
  auto s = control.read();
  CHECK(s.has_value());
  CHECK(s->battery_present == std::optional<bool>(true));
  CHECK(s->battery_mv == std::optional<uint16_t>(3996));
  CHECK(s->battery_percent == std::optional<uint8_t>(hg::estimate_liion_curve_percent(3996)));
  CHECK(s->external_power == std::optional<bool>(true));
  CHECK(s->charging == std::optional<bool>(true));
  CHECK(control.external_power() == std::optional<bool>(true));

  fake.regs[0x12] = 0x01;  // high: charged, or not charging
  CHECK(control.read()->charging == std::optional<bool>(false));

  fake.regs[0x04] = 0x04;  // battery only: never charging, whatever the pin reads
  fake.regs[0x12] = 0x00;
  s = control.read();
  CHECK(s->external_power == std::optional<bool>(false));
  CHECK(s->charging == std::optional<bool>(false));

  fake.regs[0x22] = 0x00;  // nothing on the battery node
  fake.regs[0x23] = 0x00;
  s = control.read();
  CHECK(s->battery_present == std::optional<bool>(false));
  CHECK(!s->battery_mv && !s->battery_percent && !s->charging);
}

TEST("StickS3: power off turns the amplifier off, then sends the PM1 shutdown command") {
  FakePm1 fake;
  hg::StickS3Control control(fake.read, fake.write, [](uint32_t) {});
  CHECK(control.set_amplifier(true));
  fake.written.clear();
  CHECK(control.power_off());
  CHECK_EQ(fake.regs[0x11] & 0x08, 0);
  CHECK(fake.written.back() == 0x0c);
  CHECK_EQ(fake.regs[0x0c], 0xa1);
}

TEST("AXP2101: the power key's short press is read once and cleared; enabling touches only interrupts") {
  std::array<uint8_t, 256> regs{};
  regs[0x41] = 0x40;  // another interrupt already enabled
  regs[0x49] = 0x08;  // a press from before boot
  bool fail_reads = false;
  std::vector<std::pair<uint8_t, uint8_t>> writes;
  hg::Axp2101 power(
      [&](uint8_t reg, uint8_t* out, size_t n) {
        if (fail_reads) return false;
        for (size_t i = 0; i < n; ++i) out[i] = regs[reg + i];
        return true;
      },
      [&](uint8_t reg, uint8_t value) {
        writes.emplace_back(reg, value);
        if (reg == 0x49) regs[reg] = static_cast<uint8_t>(regs[reg] & ~value);  // write 1 to clear
        else regs[reg] = value;
        return true;
      });
  CHECK(power.enable_key_press());
  CHECK_EQ(writes.size(), size_t(2));
  CHECK(writes[0] == std::make_pair(uint8_t(0x41), uint8_t(0x48)));  // short press added, the rest kept
  CHECK(writes[1] == std::make_pair(uint8_t(0x49), uint8_t(0x08)));  // the old press is dropped
  CHECK(!power.take_key_press());

  writes.clear();
  regs[0x49] = 0x0c;  // short press, plus a long press that isn't ours to clear
  CHECK(power.take_key_press());
  CHECK(!power.take_key_press());
  CHECK_EQ(writes.size(), size_t(1));
  CHECK(writes[0] == std::make_pair(uint8_t(0x49), uint8_t(0x08)));
  CHECK_EQ(regs[0x49], uint8_t(0x04));

  regs[0x49] = 0x08;
  fail_reads = true;
  CHECK(!power.take_key_press());  // a failed read is no press
  CHECK(!power.enable_key_press());
}

TEST("AXP2101: external power is read from VBUS good, and a failed read is unknown") {
  uint8_t status = 0x20;
  bool fail = false;
  hg::Axp2101 power(
      [&](uint8_t reg, uint8_t* out, size_t n) {
        if (fail || reg != 0x00 || n != 1) return false;
        *out = status;
        return true;
      },
      [&](uint8_t, uint8_t) { return false; });
  CHECK(power.vbus_good() == std::optional<bool>(true));
  status = 0x08;  // battery present, no VBUS
  CHECK(power.vbus_good() == std::optional<bool>(false));
  fail = true;
  CHECK(!power.vbus_good().has_value());
}

namespace {
// The StopWatch's M5PM1 (8-bit registers) and M5IOE1 (16-bit, low byte first) as register files.
struct FakeStopWatch {
  std::array<uint8_t, 256> pm1{};
  std::array<uint8_t, 256> ioe{};
  bool pm1_fail = false, ioe_fail = false;
  struct IoeWrite {
    uint8_t reg;
    uint16_t value;
  };
  std::vector<IoeWrite> ioe_written;
  std::vector<uint8_t> pm1_written;
  std::vector<uint32_t> delays;
  // What the expander's outputs were when each delay began.
  std::vector<uint16_t> out_at_delay;
  uint16_t ioe16(uint8_t reg) const { return static_cast<uint16_t>(ioe[reg] | (ioe[reg + 1] << 8)); }
  hg::StopWatchControl make() {
    return hg::StopWatchControl(
        {[this](uint8_t reg, uint8_t* out, size_t n) {
           if (pm1_fail) return false;
           for (size_t i = 0; i < n; ++i) out[i] = pm1[reg + i];
           if (reg == 0x48) pm1[0x48] &= 0x7F;  // the was-pressed flag clears on a read
           return true;
         },
         [this](uint8_t reg, uint8_t value) {
           if (pm1_fail) return false;
           pm1_written.push_back(reg);
           pm1[reg] = value;
           return true;
         }},
        [this](uint8_t reg, uint8_t* out, size_t n) {
          if (ioe_fail) return false;
          for (size_t i = 0; i < n; ++i) out[i] = ioe[reg + i];
          return true;
        },
        [this](uint8_t reg, uint16_t value) {
          if (ioe_fail) return false;
          ioe_written.push_back({reg, value});
          ioe[reg] = static_cast<uint8_t>(value & 0xFF);
          ioe[reg + 1] = static_cast<uint8_t>(value >> 8);
          return true;
        },
        [this](uint32_t ms) {
          delays.push_back(ms);
          out_at_delay.push_back(ioe16(0x05));
        });
  }
};
constexpr uint16_t kSwAudio = 1 << 2, kSwTouchReset = 1 << 3, kSwPanelReset = 1 << 4, kSwPanel = 1 << 7, kSwMotor = 1 << 8,
                   kSwAmp = 1 << 9;
}  // namespace

TEST("StopWatch: supplies come on, touch and panel are reset, motor and amplifier stay off") {
  FakeStopWatch fake;
  // Pins this board does not use keep whatever they had: PYG1 (the USB/UART switch) as an
  // output driven high, PYG16 open-drain.
  fake.ioe[0x03] = 0x01;
  fake.ioe[0x05] = 0x01;
  fake.ioe[0x14] = 0x80;
  fake.pm1[0x09] = 0x05;  // left by an earlier firmware: I2C idle sleep and the watchdog
  fake.pm1[0x0A] = 30;
  fake.pm1[0x49] = 0x80;  // download-mode lock, which is not this driver's to clear
  fake.ioe[0x23] = 0x35;  // I2C idle sleep after 5 s, 400 kHz and the wake bit
  fake.ioe[0x24] = 0x07;  // the LED count next to it
  auto control = fake.make();
  CHECK(control.begin());

  CHECK_EQ(fake.pm1[0x09], 0);
  CHECK_EQ(fake.pm1[0x0A], 0);
  CHECK_EQ(fake.pm1[0x49], 0x80 | 0x18 | 0x01);  // lock kept; 4 s long press; a click resets nothing
  CHECK_EQ(fake.pm1[0x43], 0x1F);                // every interrupt masked: the key is polled
  CHECK_EQ(fake.pm1[0x44], 0x3F);
  CHECK_EQ(fake.pm1[0x45], 0x07);

  const uint16_t outputs = kSwAudio | kSwTouchReset | kSwPanelReset | kSwPanel | kSwMotor | kSwAmp;
  CHECK_EQ(fake.ioe16(0x03), outputs | 0x0001);
  CHECK_EQ(fake.ioe16(0x13), 0x8000);  // our pins push-pull, PYG16 untouched
  CHECK_EQ(fake.ioe16(0x1B), 0);       // motor PWM off
  CHECK_EQ(fake.ioe[0x23], 0x30);      // idle sleep off, the other bits kept
  CHECK_EQ(fake.ioe[0x24], 0x07);
  CHECK_EQ(fake.ioe16(0x05), kSwAudio | kSwPanel | kSwTouchReset | kSwPanelReset | 0x0001);

  // 20 ms with the supplies on and the resets released, 10 ms with both resets low, 120 ms after.
  CHECK(fake.delays == std::vector<uint32_t>({20, 10, 120}));
  CHECK_EQ(fake.out_at_delay[0], kSwAudio | kSwPanel | kSwTouchReset | kSwPanelReset | 0x0001);
  CHECK_EQ(fake.out_at_delay[1], kSwAudio | kSwPanel | 0x0001);
  CHECK_EQ(fake.out_at_delay[2], kSwAudio | kSwPanel | kSwTouchReset | kSwPanelReset | 0x0001);
  // The levels are written before the pins become outputs.
  size_t out_at = 0, mode_at = 0;
  for (size_t i = 0; i < fake.ioe_written.size(); ++i) {
    if (fake.ioe_written[i].reg == 0x05 && !out_at) out_at = i + 1;
    if (fake.ioe_written[i].reg == 0x03 && !mode_at) mode_at = i + 1;
  }
  CHECK(out_at && mode_at && out_at < mode_at);
}

TEST("StopWatch: a chip that does not answer stops initialization") {
  FakeStopWatch no_pm1;
  no_pm1.pm1_fail = true;
  auto a = no_pm1.make();
  CHECK(!a.begin());
  CHECK(no_pm1.ioe_written.empty());
  CHECK(no_pm1.delays.empty());
  CHECK(!a.read());
  CHECK(!a.external_power());
  CHECK(!a.take_key_press());

  FakeStopWatch no_ioe;
  no_ioe.ioe_fail = true;
  auto b = no_ioe.make();
  CHECK(!b.begin());
  CHECK(no_ioe.delays.empty());
}

TEST("StopWatch: the amplifier is switched only on a change and leaves the other outputs alone") {
  FakeStopWatch fake;
  auto control = fake.make();
  CHECK(control.begin());
  const uint16_t before = fake.ioe16(0x05);
  const size_t writes = fake.ioe_written.size();
  CHECK(control.set_amplifier(false));  // already off
  CHECK_EQ(fake.ioe_written.size(), writes);
  CHECK(control.set_amplifier(true));
  CHECK_EQ(fake.ioe16(0x05), before | kSwAmp);
  CHECK(control.set_amplifier(true));
  CHECK_EQ(fake.ioe_written.size(), writes + 1);
  CHECK(control.set_amplifier(false));
  CHECK_EQ(fake.ioe16(0x05), before);
  fake.ioe_fail = true;
  CHECK(!control.set_amplifier(true));  // a failed write is tried again next time
  fake.ioe_fail = false;
  CHECK(control.set_amplifier(true));
  CHECK_EQ(fake.ioe16(0x05), before | kSwAmp);
}

TEST("StopWatch: cell and VIN millivolts and the active-low charge status are reported") {
  FakeStopWatch fake;
  auto control = fake.make();
  const auto set_mv = [&](uint8_t reg, uint16_t mv) {
    fake.pm1[reg] = static_cast<uint8_t>(mv & 0xFF);
    fake.pm1[reg + 1] = static_cast<uint8_t>(mv >> 8);
  };
  set_mv(0x22, 4174);
  set_mv(0x24, 5078);
  fake.pm1[0x12] = 0x03;  // CHG_STAT (PMG2) low: charging
  auto s = control.read();
  CHECK(s.has_value());
  CHECK(s->battery_present == std::optional<bool>(true));
  CHECK(s->battery_mv == std::optional<uint16_t>(4174));
  CHECK(!s->battery_percent.has_value());  // no curve was given
  CHECK(s->external_power == std::optional<bool>(true));
  CHECK(s->charging == std::optional<bool>(true));
  CHECK(control.external_power() == std::optional<bool>(true));

  fake.pm1[0x12] = 0x07;  // high: the charger rests
  CHECK(control.read()->charging == std::optional<bool>(false));

  set_mv(0x24, 0);  // on battery: never charging, whatever the pin reads
  fake.pm1[0x12] = 0x03;
  s = control.read();
  CHECK(s->external_power == std::optional<bool>(false));
  CHECK(s->charging == std::optional<bool>(false));
  CHECK(control.external_power() == std::optional<bool>(false));

  set_mv(0x22, 0);  // no cell: nothing to report about one
  s = control.read();
  CHECK(s->battery_present == std::optional<bool>(false));
  CHECK(!s->battery_mv.has_value());
  CHECK(!s->charging.has_value());

  fake.pm1_fail = true;
  CHECK(!control.read());
  CHECK(!control.external_power());
}

TEST("StopWatch: with the cell's curves the percentage follows the voltage, and resting on USB near full is 100") {
  FakeStopWatch fake;
  auto control = fake.make();
  control.use_curves(hg::kStopWatchCurve, hg::kStopWatchChargeCurve);
  const auto set_mv = [&](uint8_t reg, uint16_t mv) {
    fake.pm1[reg] = static_cast<uint8_t>(mv & 0xFF);
    fake.pm1[reg + 1] = static_cast<uint8_t>(mv >> 8);
  };
  set_mv(0x24, 0);  // on battery
  fake.pm1[0x12] = 0x07;
  set_mv(0x22, 3652);
  CHECK(control.read()->battery_percent == std::optional<uint8_t>(50));
  set_mv(0x22, 3700);  // a lighter load lifts the voltage, not the charge
  CHECK(control.read()->battery_percent == std::optional<uint8_t>(50));

  set_mv(0x24, 5078);  // plugged in and charging: the charge curve, where 3728 mV is 50
  fake.pm1[0x12] = 0x03;
  set_mv(0x22, 3806);
  CHECK(control.read()->battery_percent == std::optional<uint8_t>(62));
  fake.pm1[0x12] = 0x07;  // the charger resting far below full is a pause, not done
  CHECK(control.read()->battery_percent == std::optional<uint8_t>(62));
  CHECK(control.read()->charging == std::optional<bool>(true));
  fake.pm1[0x12] = 0x03;
  set_mv(0x22, 4176);  // the top of the charge curve
  for (int i = 0; i < 12; ++i) control.read();
  CHECK(control.read()->battery_percent == std::optional<uint8_t>(99));
  fake.pm1[0x12] = 0x07;  // full: the charger rests
  set_mv(0x22, 4124);
  CHECK(control.read()->battery_percent == std::optional<uint8_t>(100));
  CHECK(control.read()->charging == std::optional<bool>(false));
  fake.pm1[0x12] = 0x03;  // and tops the cell up again: still 100
  set_mv(0x22, 4172);
  CHECK(control.read()->battery_percent == std::optional<uint8_t>(100));
}

TEST("StopWatch: a short press of the red key is reported once, on its release; a long hold is not") {
  FakeStopWatch fake;
  auto control = fake.make();
  CHECK(!control.take_key_press());
  fake.pm1[0x48] = 0x81;  // down
  for (int i = 0; i < 3; ++i) CHECK(!control.take_key_press());
  fake.pm1[0x48] = 0x00;  // up after 300 ms
  CHECK(control.take_key_press());
  CHECK(!control.take_key_press());

  fake.pm1[0x48] = 0x80;  // pressed and released between two polls: only the flag shows it
  CHECK(control.take_key_press());
  CHECK(!control.take_key_press());

  fake.pm1[0x48] = 0x81;  // held past a second: the PM1's own long press, not a short one
  for (int i = 0; i < 15; ++i) CHECK(!control.take_key_press());
  fake.pm1[0x48] = 0x00;
  CHECK(!control.take_key_press());
  fake.pm1[0x48] = 0x81;  // the next short press counts again
  CHECK(!control.take_key_press());
  fake.pm1[0x48] = 0x00;
  CHECK(control.take_key_press());
}

TEST("StopWatch: power-off silences the amplifier, then sends the PM1's shutdown command") {
  FakeStopWatch fake;
  auto control = fake.make();
  CHECK(control.begin());
  CHECK(control.set_amplifier(true));
  CHECK(control.power_off());
  CHECK_EQ(fake.ioe16(0x05) & kSwAmp, 0);
  CHECK_EQ(fake.pm1[0x0C], 0xA1);
  CHECK_EQ(fake.pm1_written.back(), 0x0C);
}
