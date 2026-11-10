#include <cassert>
#include <iostream>
#include "../src/main.cpp"

void command(const char* text, uint32_t now) {
  fakeNow = now;
  char line[100];
  snprintf(line, sizeof(line), "%s", text);
  handleCommand(line, now);
}
void stopped() {
  assert(!running);
  assert(levels[kStandby] == LOW);
  for (int i = 0; i < 8; ++i) assert(pwmValues[i] == 0);
}
int main() {
  setup();
  stopped();
  const int expected[4][4] = {{21,19,22,23}, {16,4,17,18}, {27,14,26,25}, {32,33,35,34}};
  bool seen[40] = {};
  seen[kStandby] = true;
  for (int i = 0; i < 4; ++i) {
    const MotorPins& pins = kMotors[i];
    const int actual[4] = {pins.in1, pins.in2, pins.encA, pins.encB};
    for (int j = 0; j < 4; ++j) {
      assert(actual[j] == expected[i][j] && !seen[actual[j]]);
      seen[actual[j]] = true;
      assert(pinConfigured[actual[j]] > 0);
      assert(pinModes[actual[j]] == (j < 2 ? OUTPUT : INPUT));
      if (j >= 2) assert(interrupts[actual[j]] == 1);
    }
    assert(attachedPins[pins.pwm1] == pins.in1 && attachedPins[pins.pwm2] == pins.in2);
  }
  assert(runtimeMode == RuntimeMode::DryRun);
  command("JOG 100 5000", 0);
  stopped();
  const std::string arm = "ARM 100 " + std::to_string(dryCore.armToken());
  command(arm.c_str(), 0);
  command("DRIVE 100 1 100 50 20", 0);
  assert(dryCore.state() == drylink::State::Armed && dryCore.vx() == 100);
  stopped();  // Simulation DRIVE cannot energize hardware.
  command("MODE MOTOR_A_TEST", 0);
  assert(dryCore.state() == drylink::State::Disarmed);
  stopped();
  for (int state : {1, 3, 2, 0}) {
    levels[kMotors[0].encA] = (state >> 1) & 1;
    levels[kMotors[0].encB] = state & 1;
    encoderEdge();
  }
  assert(encoders[0].count == -4 && encoders[0].invalid == 0);
  for (int state : {2, 3, 1, 0}) {
    levels[kMotors[0].encA] = (state >> 1) & 1;
    levels[kMotors[0].encB] = state & 1;
    encoderEdge();
  }
  assert(encoders[0].count == 0 && encoders[0].invalid == 0);
  levels[kMotors[0].encA] = levels[kMotors[0].encB] = 1;
  encoderEdge();
  assert(encoders[0].count == 0 && encoders[0].invalid == 1);
  command("ENCZERO", 0);
  assert(encoders[0].count == 0 && encoders[0].invalid == 0);
  stopped();
  command("JOG 15 300", 10);
  assert(running && levels[kStandby] == HIGH);
  assert(pwmValues[kMotors[0].pwm1] == 0 && pwmValues[kMotors[0].pwm2] == 38);
  encoders[0].count = 7;
  command("ENCZERO", 20);
  assert(encoders[0].count == 7);  // resetting is rejected during a pulse
  command("JOG 20 500", 200);
  assert(startedAt == 10 && durationMs == 300);
  serviceDeadline(309);
  assert(running);
  serviceDeadline(310);
  stopped();
  command("JOG 20 300", 1309);
  stopped();
  command("JOG -20 300", 1310);
  assert(running && pwmValues[kMotors[0].pwm1] == 51 && pwmValues[kMotors[0].pwm2] == 0);
  command("STOP", 1350);
  stopped();
  for (const char* invalid : {"JOG 101 300", "JOG -101 300", "JOG 20 0",
       "JOG 20 -1", "JOG 20 2147483648", "JOG 20 4294967295", "JOG 20 99999999999999999999",
       "JOG 0 300", "JOG - 300", "JOG 2x 300", "JOG 20 3x0",
       "JOG 99999999999999999999999 300", "JOG 20 300 extra", "JOG 20", "ARM"}) {
    command(invalid, 3000);
    stopped();
  }
  command("PING", 3000);
  command("STATUS", 3000);
  command("HELP", 3000);
  assert(Serial.output.find("duty +/-1..100 percent") != std::string::npos);
  stopped();
  command("JOG 30 500", 4000);
  assert(running);
  Serial.inject("JOG 20");  // incomplete line cannot block timed stop
  fakeNow = 4500;
  loop();
  stopped();
  Serial.inject("\n" + std::string(100, 'X') + "\nPING\n");
  loop();
  loop();
  loop();
  stopped();
  assert(Serial.output.find("ERROR invalid/long line; stopped") != std::string::npos);
  assert(Serial.output.find("PONG MOTOR_A_TEST") != std::string::npos);
  hasStopped = false;
  command("JOG 15 100", UINT32_MAX - 49);
  serviceDeadline(49);
  assert(running);
  serviceDeadline(50);
  stopped();
  hasStopped = false;
  command("JOG 100 200", 6000);
  assert(running && pwmValues[kMotors[0].pwm1] == 0 && pwmValues[kMotors[0].pwm2] == 255);
  serviceDeadline(6199);
  assert(running);
  serviceDeadline(6200);
  stopped();
  command("JOG -100 500", 7200);
  assert(running && pwmValues[kMotors[0].pwm1] == 255 && pwmValues[kMotors[0].pwm2] == 0);
  command("STOP", 7201);
  stopped();
  // Extended duration runs to its deadline while the panel sends STATUS.
  command("JOG 50 5000", 9000);
  assert(running && durationMs == 5000);
  for (uint32_t now : {10000u, 11000u, 12000u, 13000u}) {
    command("STATUS", now);
    assert(running);
  }
  serviceDeadline(13999);
  assert(running);
  serviceDeadline(14000);
  stopped();
  // Lost link stops long output without waiting for the requested duration.
  command("JOG 50 60000", 15000);
  serviceDeadline(16499);
  assert(running);
  serviceDeadline(16500);
  stopped();
  assert(Serial.output.find("STOPPED link_timeout") != std::string::npos);
  // Maximum duration parses exactly, can be manually stopped, and cannot overflow.
  command("JOG -50 2147483647", 18000);
  assert(running && durationMs == 2147483647u);
  command("STOP", 18001);
  stopped();
  command("JOG 50 1", 20000);
  assert(running && durationMs == 1);
  serviceDeadline(20001);
  stopped();
  // Link-loss detection also survives the millisecond counter wrapping.
  hasStopped = false;
  command("JOG 50 5000", UINT32_MAX - 999);
  serviceDeadline(499);
  assert(running);
  serviceDeadline(500);
  stopped();
  command("JOG 100 5000", 3000);
  assert(running);
  const uint32_t oldToken = dryCore.armToken();
  command("MODE DRY_RUN", 3001);
  stopped();
  const std::string dryAck = "MODE_OK DRY_RUN STOPPED";
  const size_t ackPosition = Serial.output.rfind(dryAck);
  assert(ackPosition != std::string::npos);
  assert(Serial.output.find("\"type\":\"hello\"", ackPosition) != std::string::npos);
  assert(runtimeMode == RuntimeMode::DryRun && dryCore.armToken() != oldToken);
  command("DRIVE 100 2 200 0 0", 3002);
  assert(dryCore.state() == drylink::State::Disarmed);
  command("JOG 100 5000", 3003);
  stopped();
  command("MODE MOTOR_A_TEST", 4001);
  stopped();
  command("DRIVE 100 3 200 0 0", 4002);
  stopped();
  command("WHO", 4003);
  command("IO", 4004);
  assert(Serial.output.find("UNIFIED_ROBOT 1 mode=MOTOR_A_TEST") != std::string::npos);
  command("JOG 100 5000", 5001);
  assert(running);
  command("MODE DRY_RUN", 5002);
  stopped();
  command("MODE MOTOR_A_TEST", 7000);
  // Each pulse energizes exactly one direction on exactly one motor.
  for (int channel = 0; channel < 4; ++channel) {
    hasStopped = false;
    char text[80];
    snprintf(text, sizeof(text), "JOG %c -100 100", 'A' + channel);
    command(text, 8000 + channel*2000);
    assert(running && activeChannel == channel && levels[kStandby] == HIGH);
    // 2026-10-27 实测修正后，四路负电气方向统一为前进，正电气方向为后退。
    const int backwardPwm = kMotors[channel].pwm1;
    for (int pwm = 0; pwm < 8; ++pwm)
      assert(pwmValues[pwm] == (pwm == backwardPwm ? 255 : 0));
    command("JOG A 50 1000", 8010 + channel*2000);
    assert(activeChannel == channel);  // Busy cannot switch to another motor.
    serviceDeadline(8100 + channel*2000);
    stopped();
    hasStopped=false;
    snprintf(text, sizeof(text), "JOG %c 100 100", 'A'+channel);
    command(text,8200+channel*2000);
    const int forwardPwm = kMotors[channel].pwm2;
    for(int pwm=0;pwm<8;++pwm) assert(pwmValues[pwm] == (pwm==forwardPwm ? 255:0));
    serviceDeadline(8300+channel*2000);
    stopped();
  }
  for (int channel = 0; channel < 4; ++channel) {
    encoders[channel].count = 0;
    encoders[channel].invalid = 0;
    encoders[channel].previous = 0;
    for (int state : {1,3,2,0}) {
      levels[kMotors[channel].encA] = (state >> 1) & 1;
      levels[kMotors[channel].encB] = state & 1;
      encoderEdgeFor(channel);
    }
    assert(encoders[channel].count == -4);
  }
  command("ENCZERO C", 17000);
  assert(encoders[0].count == -4 && encoders[1].count == -4 && encoders[2].count == 0 && encoders[3].count == -4);
  command("ENCZERO ALL", 17001);
  for (int i = 0; i < 4; ++i) assert(encoders[i].count == 0);
  for (const char* text : {"JOG E 50 100", "JOG AB 50 100", "JOG B 101 100", "JOG D 50 2147483648", "JOG C 50 100 extra", "ENCZERO Z"}) {
    command(text, 18000);
    stopped();
  }
  command("CAPS", 18001);
  assert(Serial.output.find("CAPS MOTORS=ABCD PINS=FOUR_V1") != std::string::npos);
  hasStopped = false;
  command("JOG D 50 5000", 19000);
  assert(running);
  serviceDeadline(20500);
  stopped();
  command("MODE DRY_RUN", 21000);
  for (char channel : {'A','B','C','D'}) {
    char text[32];
    snprintf(text, sizeof(text), "JOG %c 100 5000", channel);
    command(text, 21001);
    stopped();
  }
  std::cout << "PASS: default-off, PWM direction, strict limits, busy rejection, "
               "cooldown, STOP, timed stop during partial input, overflow recovery, millis wrap, "
               "quadrature forward/reverse, invalid edges, encoder reset interlock, "
               "long duration with heartbeat, link-loss stop, duration overflow rejection, "
               "unified mode acknowledgement before hello, DRY_RUN output isolation, session reset on switch\n";
}
