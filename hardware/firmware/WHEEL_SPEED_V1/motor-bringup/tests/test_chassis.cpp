#include <cassert>
#include <iostream>
#include "../src/main.cpp"

void command(const char* text, uint32_t now) {
  fakeNow = now;
  char copy[100]; snprintf(copy, sizeof(copy), "%s", text);
  handleCommand(copy, now);
}
void stopped() {
  assert(!running && levels[kStandby] == LOW);
  for (int i=0; i<8; ++i) assert(pwmValues[i] == 0);
}
void expectWheels(const int expected[4]) {
  assert(running && activeChannel == -2 && levels[kStandby] == HIGH);
  for (int i=0; i<4; ++i) {
    const int measuredPolarity[4]={1,-1,-1,1};
    const int v=expected[i]*measuredPolarity[i], pwm=((v<0?-v:v)*255+50)/100;
    assert(pwmValues[kMotors[i].pwm1] == (v>0?pwm:0));
    assert(pwmValues[kMotors[i].pwm2] == (v<0?pwm:0));
  }
}
int main() {
  setup(); stopped();
  // 纯运动学：前/后/左右/旋转/斜移及饱和等比例缩放。
  struct Case { int x,y,z; int wheel[4]; };
  const Case cases[] = {
    {50,0,0,{50,50,50,50}}, {-50,0,0,{-50,-50,-50,-50}},
    {0,50,0,{-50,50,50,-50}}, {0,-50,0,{50,-50,-50,50}},
    {0,0,50,{-50,50,-50,50}}, {0,0,-50,{50,-50,50,-50}},
    {50,50,0,{0,100,100,0}}, {50,-50,0,{100,0,0,100}},
    {100,100,100,{-33,100,33,33}}, {-100,-100,-100,{33,-100,-33,-33}},
    {0,0,0,{0,0,0,0}}
  };
  mecanum::Layout normal;
  int output[4];
  for (const Case& test : cases) {
    mecanum::mix(test.x,test.y,test.z,normal,output);
    for (int i=0;i<4;++i) assert(output[i] == test.wheel[i]);
  }
  mecanum::Layout remap;
  remap.channel[0]=3; remap.channel[1]=2; remap.channel[2]=1; remap.channel[3]=0;
  remap.sign[0]=-1; remap.sign[2]=-1;
  assert(mecanum::valid(remap));
  mecanum::mix(0,50,0,remap,output);
  const int mapped[4]={-50,-50,50,50};
  for(int i=0;i<4;++i) assert(output[i] == mapped[i]);
  remap.channel[3]=3; assert(!mecanum::valid(remap));

  // 历史 ADBC 布局仍受支持；使用当前标定极性，验证轮位和电气方向各转换一次。
  mecanum::Layout measured;
  const int positions[4]={0,3,1,2};
  for(int i=0;i<4;++i) measured.channel[i]=positions[i];  // 上层保持全 +1。
  const Case calibrated[] = {
    {50,0,0,{50,-50,-50,50}}, {-50,0,0,{-50,50,50,-50}},
    {0,50,0,{-50,-50,50,50}}, {0,-50,0,{50,50,-50,-50}},
    {0,0,50,{-50,50,-50,50}}, {0,0,-50,{50,-50,50,-50}}
  };
  for(const Case& test:calibrated) {
    mecanum::mix(test.x,test.y,test.z,measured,output);
    for(int i=0;i<4;++i) assert(motorprofile::electricalPercent(i,output[i])==test.wheel[i]);
  }

  command("WHEELS 50 50 50 50 500",0); stopped();
  command("MECANUM 50 0 0 500",0); stopped();
  command("MODE MOTOR_A_TEST",0);
  command("WHEELS 50 50 50 50 500",1); stopped();
  command("MODE CHASSIS",2); stopped(); assert(!layoutConfigured);
  command("MECANUM 50 0 0 500",3); stopped();
  assert(Serial.output.find("confirm X roller layout")!=std::string::npos);
  for(const char* bad : {"LAYOUT AAAA 1 1 1 1", "LAYOUT ABCD 1 1 1 0", "LAYOUT EBCD 1 1 1 1", "LAYOUT ABCD 1 1 1", "LAYOUT ABCD 1 1 1 1 extra", "LAYOUT ABCD 1 1 1 99999999999999999", "LAYOUT ADBC -1 -1 1 1"}) {
    command(bad,4); stopped(); assert(!layoutConfigured);
  }
  command("LAYOUT ABCD 1 1 1 1",5); assert(layoutConfigured); stopped();
  for (const Case& test : cases) {
    if (!test.x && !test.y && !test.z) continue;
    hasStopped=false;
    char text[90]; snprintf(text,sizeof(text),"MECANUM %d %d %d 100",test.x,test.y,test.z);
    command(text,1000); expectWheels(test.wheel);
    command("STATUS",1010); assert(Serial.output.find("RUNNING CHASSIS")!=std::string::npos);
    command("LAYOUT DCBA 1 1 1 1",1011); assert(wheelLayout.channel[0]==0);
    command("MECANUM 100 0 0 2000",1012); assert(startedAt==1000 && durationMs==100);
    serviceDeadline(1100); stopped();
    command("MECANUM 50 0 0 100",1101); stopped();  // 保留停后冷却。
  }
  hasStopped=false;
  command("LAYOUT DCBA 1 1 1 1",3000);
  const int layoutMapped[4]={-50,50,50,-50};
  command("MECANUM 0 50 0 5000",3001); expectWheels(layoutMapped);
  command("STOP",3002); stopped();
  hasStopped=false;
  command("WHEELS 1 1 1 1 500",3100);
  const int allForward[4]={1,1,1,1}; expectWheels(allForward);
  command("STOP",3101); stopped();
  hasStopped=false;
  command("WHEELS -1 -1 -1 -1 500",3200);
  const int allBackward[4]={-1,-1,-1,-1}; expectWheels(allBackward);
  command("STOP",3201); stopped();
  hasStopped=false;
  command("WHEELS 100 -50 0 25 5000",4000);
  const int direct[4]={100,-50,0,25}; expectWheels(direct);
  serviceDeadline(5500); stopped();
  hasStopped=false;
  command("WHEELS 100 -50 0 25 5000",6000);
  command("WHEELS 0 0 0 0 1",6001); stopped();
  for(const char* bad : {"MECANUM 101 0 0 500", "MECANUM 0 -101 0 500", "MECANUM 1 2 3 0", "MECANUM 1 2 3 -1", "MECANUM 1 2 3 2147483648", "MECANUM 1 2 3 99999999999999999999999999", "MECANUM 1x 2 3 5", "MECANUM 1 2 3 5 extra", "WHEELS 100 -100 0 101 100", "WHEELS 0 1 2 3", "WHEELS + 1 2 3 500"}) {
    hasStopped=false; command(bad,8000); stopped();
  }
  hasStopped=false;
  command("WHEELS 1 2 3 4 100",UINT32_MAX-49);
  serviceDeadline(49); assert(running);
  serviceDeadline(50); stopped();
  hasStopped=false;
  command("WHEELS 10 20 30 40 2147483647",10000);
  assert(running && durationMs==2147483647u);
  command("PING",11000); serviceDeadline(12000); assert(running);
  command("MODE DRY_RUN",12001); stopped(); assert(!layoutConfigured);
  const std::string arm="ARM 234 "+std::to_string(dryCore.armToken());
  command(arm.c_str(),12002); command("DRIVE 234 1 100 0 0",12003); stopped();
  command("MODE CHASSIS",14000); stopped(); assert(!layoutConfigured);
  command("MECANUM 50 0 0 500",14001); stopped();
  command("CAPS",14002);
  assert(Serial.output.find("CHASSIS_CAPS API=MECANUM_V2 MIX=X UNITS=PERCENT FORWARD=UNIFIED")!=std::string::npos);
  assert(Serial.output.find("CHASSIS_HOLD_RELEASE GUARD_MS=60") != std::string::npos);
  command("POLARITY",14003); stopped();
  assert(Serial.output.find("MOTOR_FORWARD A=1 B=-1 C=-1 D=1")!=std::string::npos);
  // 2026-10-27 最新轮位 CBAD；B/C 的修正集中在底层，上层仍然全 +1。
  command("LAYOUT CBAD 1 1 1 1",15000);
  assert(layoutConfigured && wheelLayout.channel[0]==2 && wheelLayout.channel[1]==1 &&
         wheelLayout.channel[2]==0 && wheelLayout.channel[3]==3);
  const Case currentLayout[] = {
    {50,0,0,{50,50,50,50}}, {-50,0,0,{-50,-50,-50,-50}},
    {0,50,0,{50,50,-50,-50}}, {0,-50,0,{-50,-50,50,50}},
    {0,0,50,{-50,50,-50,50}}, {0,0,-50,{50,-50,50,-50}},
    {50,-50,0,{0,0,100,100}}, {50,50,0,{100,100,0,0}}
  };
  uint32_t testAt=16000;
  for(const Case& test:currentLayout) {
    char text[80]; snprintf(text,sizeof(text),"MECANUM %d %d %d 100",test.x,test.y,test.z);
    command(text,testAt); expectWheels(test.wheel);
    serviceDeadline(testAt+100); stopped();
    testAt+=2000;
  }
  command("MECANUM_HOLD 50 0 0", 33000); expectWheels(currentLayout[0].wheel);
  assert(holdMotion && durationMs == 250);
  command("MECANUM_HOLD 100 0 0", 33200);
  const int fullForward[4] = {100,100,100,100}; expectWheels(fullForward);
  assert(startedAt == 33200);
  command("MECANUM_HOLD -100 0 0", 33300); expectWheels(fullForward);
  command("MECANUM_HOLD 100 50 0", 33301); expectWheels(fullForward);
  assert(startedAt == 33200);
  command("PING", 33449); assert(running);
  serviceDeadline(33450); stopped(); assert(!holdMotion);
  command("MECANUM_HOLD 50 0 0", 33460); stopped();
  command("MECANUM_HOLD 50 0 0", 34500); assert(holdMotion);
  command("STOP", 34501); stopped(); assert(!holdMotion);
  command("MECANUM_HOLD 50 0 0", 34561); stopped();
  command("MECANUM_HOLD 50 0 0", 35501); expectWheels(currentLayout[0].wheel);
  command("STOP HOLD", 35600); stopped();
  assert(Serial.output.find("STOPPED hold_release") != std::string::npos);
  command("MECANUM_HOLD -50 0 0", 35659); stopped();
  assert(Serial.output.find("ERROR hold release guard") != std::string::npos);
  command("MECANUM -50 0 0 100", 35660); stopped();  // 定时动作仍需等待 1 秒。
  command("MECANUM_HOLD -50 0 0", 35660); expectWheels(currentLayout[1].wheel);
  command("STOP HOLD", 35760); stopped();
  command("STOP", 35761); stopped();  // 紧急停车撤销短间隔资格。
  command("MECANUM_HOLD 50 0 0", 35820); stopped();
  command("MECANUM 50 0 0 100", 36760); expectWheels(currentLayout[0].wheel);
  command("STOP HOLD", 36761); expectWheels(currentLayout[0].wheel);
  serviceDeadline(36860); stopped();
  command("MECANUM_MANUAL 50 0 0", 36861); expectWheels(currentLayout[0].wheel);
  command("MECANUM_MANUAL -50 0 0", 36862); expectWheels(currentLayout[1].wheel);
  command("MECANUM_MANUAL 0 0 0", 36863); stopped();
  command("MECANUM_MANUAL 0 50 0", 36863); expectWheels(currentLayout[2].wheel);
  command("MECANUM 50 0 0 100", 36864); expectWheels(currentLayout[2].wheel);
  command("MECANUM_KEEP", 37000); expectWheels(currentLayout[2].wheel);
  command("MECANUM_KEEP", 37250); stopped();  // 达到续期时限后，迟到续期不能重新启动。
  assert(Serial.output.find("STOPPED manual_timeout") != std::string::npos);
  command("MECANUM_MANUAL 50 0 0", 37251); expectWheels(currentLayout[0].wheel);
  command("MECANUM_MANUAL 50 50 0", 37252); expectWheels(currentLayout[0].wheel);
  command("MECANUM_MANUAL 0 0 0", 37253); stopped();
  command("MECANUM 50 0 0 100", 37254); stopped();
  command("MECANUM_STREAM 20 0 0 151", 37255); stopped();
  command("MECANUM_STREAM 20 20 1 150", 37256); stopped();
  command("MECANUM_STREAM 20 5 3 150", 37257);
  const int streamMixed[4] = {22,28,12,18}; expectWheels(streamMixed);
  assert(streamMotion && Serial.output.find("CHASSIS_STREAM LEASE_MS=150") != std::string::npos);
  command("MECANUM_STREAM -20 -5 -3 75", 37357);
  const int streamReverse[4] = {-22,-28,-12,-18}; expectWheels(streamReverse);
  command("PING", 37431); expectWheels(streamReverse);
  // A target arriving at the lease deadline must not re-energize the driver.
  command("MECANUM_STREAM 20 5 3 150", 37432); stopped();
  assert(streamLocked && Serial.output.find("STOPPED stream_timeout") != std::string::npos);
  command("STOP", 37433); stopped();
  command("MECANUM_STREAM 20 5 3 150", 37434); stopped();
  command("MODE CHASSIS", 37435);
  command("LAYOUT CBAD 1 1 1 1", 37436);
  command("MECANUM_STREAM 20 5 3 150", 37437); expectWheels(streamMixed);
  command("STOP", 37438); stopped();
  command("MECANUM_STREAM 20 5 3 150", 37439); expectWheels(streamMixed);
  command("STOP", 37440); stopped();
  command("SPEED_STREAM 600 200 50 150", 37441); stopped();  // No encoder calibration.
  calibrationData.valid = true;
  calibrationData.vmax = 9000;
  for (auto& wheel : calibrationData.wheel) { wheel.sign = 1; wheel.a = 5; wheel.b = .015f; }
  command("SPEED_INFO", 37442);
  assert(Serial.output.find("SPEED_INFO 7200 1 1 1 1") != std::string::npos);
  command("SPEED_STREAM 7100 100 1 150", 37443); stopped();
  fakeMicros = 37444000;
  command("SPEED_STREAM 600 200 50 150", 37444);
  assert(running && streamMotion && closedLoopActive);
  assert(goalTarget[0] == 750 && goalTarget[1] == 850 && goalTarget[2] == 350 && goalTarget[3] == 450);
  serviceControl(fakeMicros + 10000);
  const float previousTarget = rampedTarget[0];
  const float previousDuty = appliedDuty[0];
  assert(previousTarget > 0 && previousDuty > 0 && previousDuty <= 40);
  command("SPEED_STREAM 650 200 50 80", 37460);
  assert(goalTarget[0] == 800 && rampedTarget[0] == previousTarget && appliedDuty[0] == previousDuty);
  command("MECANUM_STREAM 20 0 0 150", 37461);
  assert(closedLoopActive && goalTarget[0] == 800);  // No mode swap while driving.
  command("SPEED_STREAM 650 200 50 80", 37540); stopped();
  assert(streamLocked && !closedLoopActive);
  command("ENC_SNAPSHOT", 37541);
  assert(Serial.output.find("ENC_SNAPSHOT ") != std::string::npos);
  command("CAPS", 37542);
  assert(Serial.output.find("CHASSIS_SPEED_PWM_MAX=80") != std::string::npos);
  command("MODE CHASSIS", 37543);
  command("LAYOUT CBAD 1 1 1 1", 37544);
  fakeMicros = 37545000;
  command("SPEED_STREAM 7200 0 0 150", 37545);
  const uint32_t driveStart = fakeMicros;
  for (uint32_t i = 1; i <= 12; ++i) {
    fakeMicros = driveStart + i * 10000;
    fakeNow = fakeMicros / 1000;
    serviceControl(fakeMicros);
    for (float duty : appliedDuty) assert(duty >= 0 && duty <= 80);
  }
  const int streamMaxDuty[4] = {80, 80, 80, 80};
  expectWheels(streamMaxDuty);  // This target needs more than the previous 40% PWM.
  command("STOP", 37666); stopped();
  std::cout << "PASS: X kinematics, scaling, remap/polarity, mode/layout gates, four-wheel PWM, strict parsing, busy/cooldown, zero STOP, timers/wrap/watchdog, DRY_RUN isolation\n";
}
