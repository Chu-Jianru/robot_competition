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
    const int measuredPolarity[4]={-1,-1,-1,-1};
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
    {50,0,0,{-50,-50,-50,-50}}, {-50,0,0,{50,50,50,50}},
    {0,50,0,{50,-50,50,-50}}, {0,-50,0,{-50,50,-50,50}},
    {0,0,50,{50,50,-50,-50}}, {0,0,-50,{-50,-50,50,50}}
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
  command("POLARITY",14003); stopped();
  assert(Serial.output.find("MOTOR_FORWARD A=-1 B=-1 C=-1 D=-1")!=std::string::npos);
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
  std::cout << "PASS: X kinematics, scaling, remap/polarity, mode/layout gates, four-wheel PWM, strict parsing, busy/cooldown, zero STOP, timers/wrap/watchdog, DRY_RUN isolation\n";
}
