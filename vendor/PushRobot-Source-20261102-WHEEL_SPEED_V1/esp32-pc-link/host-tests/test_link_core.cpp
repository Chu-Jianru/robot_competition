// Host-side tests for the DRY_RUN protocol core. No ESP32 or motor hardware
// is involved. Build with g++ (see run_tests.sh / run_tests.ps1).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "link_core.h"

namespace {

int g_pass = 0;
int g_fail = 0;

void checkTrue(bool condition, const char* expression, int line) {
  if (condition) {
    ++g_pass;
  } else {
    ++g_fail;
    std::printf("FAIL line %d: %s\n", line, expression);
  }
}

void checkContains(const std::string& text, const char* needle, int line) {
  if (text.find(needle) != std::string::npos) {
    ++g_pass;
  } else {
    ++g_fail;
    std::printf("FAIL line %d: missing [%s] in [%s]\n", line, needle,
                text.c_str());
  }
}

#define CHECK(cond) checkTrue((cond), #cond, __LINE__)
#define CHECK_CONTAINS(text, needle) \
  checkContains((text), (needle), __LINE__)

using drylink::Error;
using drylink::LinkCore;
using drylink::State;
using drylink::StopReason;

struct Fixture {
  LinkCore core;
  std::vector<std::string> out;

  explicit Fixture(uint32_t boot = 0x11111111u, uint32_t token = 0x22222222u,
                   uint32_t now = 1000) {
    LinkCore::Config cfg;
    cfg.boot_id = boot;
    cfg.arm_token = token;
    cfg.random = nullptr;
    cfg.random_ctx = nullptr;
    cfg.emit = &Fixture::emitLine;
    cfg.emit_ctx = this;
    core.begin(now, cfg);
    out.clear();
  }

  static void emitLine(void* ctx, const char* line) {
    static_cast<Fixture*>(ctx)->out.emplace_back(line);
  }

  void clear() { out.clear(); }

  const std::string& last() const {
    static const std::string empty;
    return out.empty() ? empty : out.back();
  }

  void feed(const std::string& text, uint32_t now) {
    core.feed(reinterpret_cast<const uint8_t*>(text.data()), text.size(), now);
  }

  std::string armCommand(uint32_t session) const {
    return std::string("ARM ") + std::to_string(session) + " " +
           std::to_string(core.armToken()) + "\n";
  }
};

void testHello() {
  LinkCore core;
  std::vector<std::string> out;
  LinkCore::Config cfg;
  cfg.boot_id = 0xB00B1234u;
  cfg.arm_token = 0xCAFE5678u;
  cfg.random = nullptr;
  cfg.random_ctx = nullptr;
  cfg.emit = [](void* ctx, const char* line) {
    static_cast<std::vector<std::string>*>(ctx)->emplace_back(line);
  };
  cfg.emit_ctx = &out;
  core.begin(5000, cfg);

  CHECK(out.size() == 1u);
  CHECK_CONTAINS(out[0], "\"type\":\"hello\"");
  CHECK_CONTAINS(out[0], "\"fw\":\"0.1.0\"");
  CHECK_CONTAINS(out[0], "\"mode\":\"DRY_RUN\"");
  CHECK_CONTAINS(out[0], "\"protocol\":1");
  CHECK_CONTAINS(out[0], "\"boot_id\":");
  CHECK_CONTAINS(out[0], "\"arm_token\":");
  CHECK_CONTAINS(out[0], "\"watchdog_ms\":300");
  CHECK_CONTAINS(out[0], "\"state\":\"DISARMED\"");
}

void testPingFragmented() {
  Fixture f;
  f.feed("P", 1000);
  f.feed("I", 1010);
  f.feed("N", 1020);
  f.feed("G", 1030);
  f.feed("\n", 1040);
  CHECK(f.out.size() == 1u);
  CHECK_CONTAINS(f.last(), "\"type\":\"pong\"");
  CHECK_CONTAINS(f.last(), "\"boot_id\":");
}

void testCoalescedLinesAndCrLf() {
  Fixture f;
  f.feed("PING\nSTATUS\n", 1000);
  CHECK(f.out.size() == 2u);
  CHECK_CONTAINS(f.out[0], "\"type\":\"pong\"");
  CHECK_CONTAINS(f.out[1], "\"type\":\"status\"");
  f.clear();
  f.feed("PING\r\n", 1010);
  CHECK(f.out.size() == 1u);
  CHECK_CONTAINS(f.last(), "\"type\":\"pong\"");
}

void testEmptyAndWhitespaceLinesIgnored() {
  Fixture f;
  f.feed("\n \t \n\r\n", 1000);
  CHECK(f.out.empty());
  f.feed("PING\n", 1010);
  CHECK(f.out.size() == 1u);
}

void testTooLongResync() {
  Fixture f;
  std::string long_line(300, 'A');
  long_line += "\n";
  f.feed(long_line, 1000);
  CHECK(f.out.size() == 1u);
  CHECK_CONTAINS(f.last(), "TOO_LONG");
  f.feed("PING\n", 1010);
  CHECK(f.out.size() == 2u);
  CHECK_CONTAINS(f.last(), "\"type\":\"pong\"");
}

void testBadCharacterResync() {
  Fixture f;
  std::string line = "PI";
  line.push_back('\x01');
  line += "NG\n";
  f.feed(line, 1000);
  CHECK(f.out.size() == 1u);
  CHECK_CONTAINS(f.last(), "BAD_LINE");
  f.feed("PING\n", 1010);
  CHECK(f.out.size() == 2u);
  CHECK_CONTAINS(f.last(), "\"type\":\"pong\"");
}

void testUnknownAndBadArgs() {
  Fixture f;
  f.feed("FOO\n", 1000);
  CHECK_CONTAINS(f.last(), "UNKNOWN_CMD");
  f.feed("PING 1\n", 1010);
  CHECK_CONTAINS(f.last(), "BAD_ARGS");
  f.feed("STATUS extra\n", 1020);
  CHECK_CONTAINS(f.last(), "BAD_ARGS");
  f.feed("ARM 100\n", 1030);
  CHECK_CONTAINS(f.last(), "BAD_ARGS");
  f.feed("DRIVE 1 2 3\n", 1040);
  CHECK_CONTAINS(f.last(), "BAD_ARGS");
  f.feed("DRIVE 1 1 +10 0 0\n", 1050);
  CHECK_CONTAINS(f.last(), "BAD_ARGS");
  f.feed("DRIVE 1 1 999999999999999 0 0\n", 1060);
  CHECK_CONTAINS(f.last(), "BAD_ARGS");
}

void testDriveBeforeArm() {
  Fixture f;
  f.feed("DRIVE 1 1 0 0 0\n", 1000);
  CHECK_CONTAINS(f.last(), "NOT_ARMED");
  CHECK(f.core.state() == State::Disarmed);
}

void testArmTokenSessionAndStop() {
  Fixture f;
  const uint32_t token1 = f.core.armToken();
  f.feed(f.armCommand(100), 1000);
  CHECK(f.core.state() == State::Armed);
  CHECK_CONTAINS(f.last(), "\"cmd\":\"ARM\"");
  CHECK_CONTAINS(f.last(), "\"ok\":true");
  CHECK(f.core.session() == 100u);

  f.feed(f.armCommand(101), 1010);
  CHECK_CONTAINS(f.last(), "ALREADY_ARMED");

  f.feed("DRIVE 100 1 100 -20 10\n", 1100);
  CHECK_CONTAINS(f.last(), "\"cmd\":\"DRIVE\"");
  CHECK_CONTAINS(f.last(), "\"ok\":true");
  CHECK(f.core.vx() == 100);
  CHECK(f.core.vy() == -20);
  CHECK(f.core.w() == 10);
  CHECK(f.core.lastSeq() == 1u);

  f.feed("DRIVE 100 1 0 0 0\n", 1150);
  CHECK_CONTAINS(f.last(), "STALE_SEQ");
  CHECK(f.core.vx() == 100);

  f.feed("STOP\n", 1200);
  CHECK(f.core.state() == State::Disarmed);
  CHECK_CONTAINS(f.last(), "\"cmd\":\"STOP\"");
  CHECK(f.core.armToken() != token1);

  const uint32_t token2 = f.core.armToken();
  f.feed(std::string("ARM 100 ") + std::to_string(token1) + "\n", 1300);
  CHECK_CONTAINS(f.last(), "STALE_ARM_TOKEN");

  f.feed(std::string("ARM 100 ") + std::to_string(token2) + "\n", 1310);
  CHECK_CONTAINS(f.last(), "SESSION_REUSED");

  f.feed(std::string("ARM 101 ") + std::to_string(token2) + "\n", 1320);
  CHECK(f.core.state() == State::Armed);
  CHECK(f.core.session() == 101u);
  CHECK(f.core.lastSeq() == 0u);
}

void testSessionMismatchDoesNotRefresh() {
  Fixture f;
  f.feed(f.armCommand(100), 1000);
  f.feed("DRIVE 101 1 10 0 0\n", 1299);
  CHECK_CONTAINS(f.last(), "SESSION_MISMATCH");
  CHECK(f.core.lastSeq() == 0u);
  f.core.tick(1300);
  CHECK(f.core.state() == State::Disarmed);
  CHECK(f.core.reason() == StopReason::Watchdog);
}

void testRangeAndBoundaries() {
  Fixture f;
  f.feed(f.armCommand(100), 1000);

  f.feed("DRIVE 100 1 300 -300 180\n", 1100);
  CHECK_CONTAINS(f.last(), "\"ok\":true");
  CHECK(f.core.vx() == 300);
  CHECK(f.core.vy() == -300);
  CHECK(f.core.w() == 180);

  f.feed("DRIVE 100 2 301 0 0\n", 1110);
  CHECK_CONTAINS(f.last(), "RANGE");
  f.feed("DRIVE 100 3 0 301 0\n", 1120);
  CHECK_CONTAINS(f.last(), "RANGE");
  f.feed("DRIVE 100 4 0 0 -181\n", 1130);
  CHECK_CONTAINS(f.last(), "RANGE");
  CHECK(f.core.lastSeq() == 1u);
}

void testRangeDoesNotRefreshWatchdog() {
  Fixture f;
  f.feed(f.armCommand(100), 4000);
  f.feed("DRIVE 100 1 301 0 0\n", 4299);
  CHECK_CONTAINS(f.last(), "RANGE");
  f.core.tick(4300);
  CHECK(f.core.state() == State::Disarmed);
  CHECK(f.core.reason() == StopReason::Watchdog);
}

void testZeroDriveRefreshesWatchdog() {
  Fixture f;
  f.feed(f.armCommand(200), 2000);
  f.feed("DRIVE 200 1 0 0 0\n", 2200);
  CHECK_CONTAINS(f.last(), "\"ok\":true");
  f.core.tick(2499);
  CHECK(f.core.state() == State::Armed);
  f.core.tick(2500);
  CHECK(f.core.state() == State::Disarmed);
  CHECK(f.core.reason() == StopReason::Watchdog);
  CHECK_CONTAINS(f.last(), "\"event\":\"WATCHDOG_STOP\"");
  CHECK_CONTAINS(f.last(), "\"reason\":\"WATCHDOG\"");
}

void testPingAndStatusDoNotRefreshWatchdog() {
  Fixture f;
  f.feed(f.armCommand(300), 3000);
  f.feed("PING\n", 3299);
  f.core.tick(3300);
  CHECK(f.core.state() == State::Disarmed);

  Fixture g;
  g.feed(g.armCommand(301), 3000);
  g.feed("STATUS\n", 3299);
  g.core.tick(3300);
  CHECK(g.core.state() == State::Disarmed);
}

void testExactBoundaryFeedRejectsDrive() {
  Fixture f;
  f.feed(f.armCommand(600), 6000);
  // feed() checks the watchdog before processing bytes, so a DRIVE landing
  // exactly at 300 ms is rejected.
  f.feed("DRIVE 600 1 10 0 0\n", 6300);
  CHECK(f.core.state() == State::Disarmed);
  CHECK_CONTAINS(f.last(), "NOT_ARMED");
  CHECK(f.core.lastSeq() == 0u);
}

void testWatchdogWrapAround() {
  Fixture f(0x11111111u, 0x22222222u, 0xFFFFFF00u);
  f.feed(f.armCommand(700), 0xFFFFFF00u);
  f.feed("DRIVE 700 1 10 0 0\n", 0xFFFFFF20u);
  f.core.tick(0x0000004Bu);  // 299 ms after last drive
  CHECK(f.core.state() == State::Armed);
  f.core.tick(0x0000004Cu);  // 300 ms
  CHECK(f.core.state() == State::Disarmed);
  CHECK(f.core.reason() == StopReason::Watchdog);
}

void testStatusFields() {
  Fixture f;
  f.feed(f.armCommand(100), 1000);
  f.feed("DRIVE 100 7 50 -20 10\n", 1100);
  f.feed("STATUS\n", 1200);
  CHECK_CONTAINS(f.last(), "\"type\":\"status\"");
  CHECK_CONTAINS(f.last(), "\"state\":\"ARMED\"");
  CHECK_CONTAINS(f.last(), "\"session\":100");
  CHECK_CONTAINS(f.last(), "\"last_seq\":7");
  CHECK_CONTAINS(f.last(), "\"vx\":50");
  CHECK_CONTAINS(f.last(), "\"vy\":-20");
  CHECK_CONTAINS(f.last(), "\"w\":10");
  CHECK_CONTAINS(f.last(), "\"age_ms\":100");
  CHECK_CONTAINS(f.last(), "\"watchdog_ms\":300");
  CHECK_CONTAINS(f.last(), "\"dry_run\":true");

  Fixture g;
  g.feed("STATUS\n", 1000);
  CHECK_CONTAINS(g.last(), "\"state\":\"DISARMED\"");
  CHECK_CONTAINS(g.last(), "\"session\":0");
  CHECK_CONTAINS(g.last(), "\"age_ms\":-1");
  CHECK_CONTAINS(g.last(), "\"reason\":\"BOOT\"");
}

void testStopWithExtraTokensStillStops() {
  Fixture f;
  f.feed(f.armCommand(100), 1000);
  f.feed("STOP now please\n", 1100);
  CHECK(f.core.state() == State::Disarmed);
  CHECK_CONTAINS(f.last(), "\"cmd\":\"STOP\"");
  CHECK_CONTAINS(f.last(), "\"ok\":true");
}

void testDisarm() {
  Fixture f;
  f.feed(f.armCommand(100), 1000);
  f.feed("DISARM\n", 1100);
  CHECK(f.core.state() == State::Disarmed);
  CHECK(f.core.reason() == StopReason::Disarm);
  CHECK_CONTAINS(f.last(), "\"cmd\":\"DISARM\"");
  CHECK_CONTAINS(f.last(), "\"reason\":\"DISARM\"");
}

void testTokenRotatesOnEveryStop() {
  Fixture f;
  uint32_t previous = f.core.armToken();
  for (int i = 0; i < 5; ++i) {
    f.feed("STOP\n", 1000 + static_cast<uint32_t>(i) * 10);
    const uint32_t current = f.core.armToken();
    CHECK(current != 0u);
    CHECK(current != previous);
    previous = current;
  }
}

void testRebootRejectsOldArmAndOldDrive() {
  Fixture old_core;
  old_core.feed(old_core.armCommand(100), 1000);
  const std::string old_arm = old_core.armCommand(100);
  const std::string old_drive = "DRIVE 100 1 50 0 0\n";

  Fixture after_reboot(0x33333333u, 0x44444444u, 2000);
  after_reboot.feed(old_arm, 2000);
  CHECK_CONTAINS(after_reboot.last(), "STALE_ARM_TOKEN");
  after_reboot.feed(old_drive, 2010);
  CHECK_CONTAINS(after_reboot.last(), "NOT_ARMED");
  after_reboot.feed(after_reboot.armCommand(200), 2020);
  CHECK(after_reboot.core.state() == State::Armed);
  CHECK(after_reboot.core.session() == 200u);
}

void testZeroIdsRejected() {
  Fixture f;
  f.feed(std::string("ARM 0 ") + std::to_string(f.core.armToken()) + "\n", 1000);
  CHECK_CONTAINS(f.last(), "BAD_ARGS");
  f.feed(f.armCommand(100), 1010);
  f.feed("DRIVE 100 0 0 0 0\n", 1020);
  CHECK_CONTAINS(f.last(), "BAD_ARGS");
}

void testTabsAccepted() {
  Fixture f;
  f.feed(std::string("ARM\t100\t") + std::to_string(f.core.armToken()) + "\n",
         1000);
  CHECK(f.core.state() == State::Armed);
  f.feed("DRIVE\t100\t1\t10\t0\t0\n", 1100);
  CHECK_CONTAINS(f.last(), "\"ok\":true");
  f.feed("PING\t\n", 1200);
  CHECK_CONTAINS(f.last(), "\"type\":\"pong\"");
}

}  // namespace

int main() {
  testHello();
  testPingFragmented();
  testCoalescedLinesAndCrLf();
  testEmptyAndWhitespaceLinesIgnored();
  testTooLongResync();
  testBadCharacterResync();
  testUnknownAndBadArgs();
  testDriveBeforeArm();
  testArmTokenSessionAndStop();
  testSessionMismatchDoesNotRefresh();
  testRangeAndBoundaries();
  testRangeDoesNotRefreshWatchdog();
  testZeroDriveRefreshesWatchdog();
  testPingAndStatusDoNotRefreshWatchdog();
  testExactBoundaryFeedRejectsDrive();
  testWatchdogWrapAround();
  testStatusFields();
  testStopWithExtraTokensStillStops();
  testDisarm();
  testTokenRotatesOnEveryStop();
  testRebootRejectsOldArmAndOldDrive();
  testZeroIdsRejected();
  testTabsAccepted();

  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
