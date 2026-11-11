"""Run the production initializer across power-on and protocol resets."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
import unittest
from test_output_channel_lifetime import extract


class MaslowBeginOnceTest(unittest.TestCase):
    def test_soft_reset_preserves_references_and_does_not_drive_self_test(self):
        active=Path(__file__).resolve().parents[3]
        root=Path(os.environ.get('MASLOW_BEGIN_SOURCE_ROOT',str(active)))
        source=(root/'FluidNC/src/Maslow/Maslow.cpp').read_text(encoding='utf-8-sig')
        method=extract(source,'void Maslow_::begin(')
        direction_flags='\n'.join(re.findall(r'^static constexpr bool [a-z]+EncoderReversed = (?:true|false);',source,re.MULTILINE))
        harness=r'''
#include <cstdlib>
#include <iostream>
#include <string>
constexpr int _TL=0,_TR=1,_BL=2,_BR=3,LOW=0,OUTPUT=1,INPUT=0,TCAADDR=70;
constexpr int tlIn1Pin=1,tlIn2Pin=2,tlADCPin=3,TLEncoderLine=2,tlIn1Channel=0,tlIn2Channel=1;
constexpr int trIn1Pin=4,trIn2Pin=5,trADCPin=6,TREncoderLine=1,trIn1Channel=2,trIn2Channel=3;
constexpr int blIn1Pin=7,blIn2Pin=8,blADCPin=9,BLEncoderLine=3,blIn1Channel=4,blIn2Channel=5;
constexpr int brIn1Pin=10,brIn2Pin=11,brADCPin=12,BREncoderLine=0,brIn1Channel=6,brIn2Channel=7;
DIRECTION_FLAGS
constexpr int coolingFanPin=47,ETHERNETLEDPIN=39,WIFILED=35,REDLED=14,SERVOFAULT=40;
#define log_error(...) ((void)0)
#define log_info(...) ((void)0)
struct WireStub {int begins=0;void begin(int,int,int){++begins;}void setTimeOut(int){}} Wire;
unsigned long now=100;
unsigned long millis(){return now;}
void pinMode(int,int){}
void digitalWrite(int,int){}
void record_initial_fault();
struct Axis {
 int begins=0,zeroes=0,motor_tests=0;double belt=0;bool reversed=false;
 void begin(int,int,int,int,int,int,bool reverseValue=false){
  ++begins;++zeroes;++motor_tests;belt=0;reversed=reverseValue;record_initial_fault();
 }
};
struct Calibration {
 bool axisHomed[4]={true,true,true,true};int centers=0,retractCurrentThreshold=850;
 void updateCenterXY(){++centers;}
};
struct Mux {int begins=0;void begin(int,WireStub&){++begins;}};
struct Maslow_ {
 Axis axis[4];Calibration calibration;Mux I2CMux;
 bool error=false,watchdogFired=false;std::string errorMessage;
 int nvs_z_loads=0,nvs_belt_loads=0,stops=0,watchdog_refreshes=0;
 unsigned long lastCallToUpdate=0;
 void begin(void(*)());
 void loadZPos(){++nvs_z_loads;}
 void loadBeltPositions(){++nvs_belt_loads;for(int i=0;i<4;++i)axis[i].belt=100+i;}
 void stopMotors(){++stops;}
 void resetUpdateWatchdog(){++watchdog_refreshes;lastCallToUpdate=millis();}
} Maslow;
bool initial_fault=false;
void record_initial_fault(){if(initial_fault){Maslow.error=true;Maslow.errorMessage="initial motor failure";}}
void check(bool ok,const char* message){if(!ok){std::cerr<<"FAIL "<<message<<"\n";std::exit(2);}}
METHOD
int main(int argc,char**){
 initial_fault=argc>1;
 Maslow.begin(nullptr);
 const bool expected[]={tlEncoderReversed,trEncoderReversed,blEncoderReversed,brEncoderReversed};
 for(int i=0;i<4;i++)check(Maslow.axis[i].reversed==expected[i],"encoder reversal flag not passed to axis");
 check(Wire.begins==1 && Maslow.I2CMux.begins==1,"power-on bus initialization missing");
 check(Maslow.nvs_z_loads==1 && Maslow.nvs_belt_loads==1,"power-on NVS restore missing");
 for(int i=0;i<4;++i){
  check(Maslow.axis[i].begins==1 && Maslow.axis[i].motor_tests==1 && Maslow.axis[i].zeroes==1,"power-on axis initialization count");
  check(Maslow.axis[i].belt==100+i,"power-on restored belt reference lost");
 }
 if(initial_fault)check(Maslow.error,"initial hardware error discarded");
 for(int i=0;i<4;++i){Maslow.axis[i].belt=600+17*i;Maslow.calibration.axisHomed[i]=true;}
 Maslow.error=true;Maslow.watchdogFired=true;Maslow.errorMessage="latched stop";
 for(int reset=0;reset<3;++reset){
  now+=1000;Maslow.begin(nullptr);
  check(Wire.begins==1 && Maslow.I2CMux.begins==1,"soft reset reinitialized hardware buses");
  check(Maslow.nvs_z_loads==1 && Maslow.nvs_belt_loads==1,"soft reset loaded stale NVS");
  for(int i=0;i<4;++i){
   check(Maslow.axis[i].begins==1 && Maslow.axis[i].zeroes==1 && Maslow.axis[i].motor_tests==1,"soft reset repeated motor self-test/zero");
   check(Maslow.axis[i].belt==600+17*i,"soft reset changed real belt reference");
   check(Maslow.calibration.axisHomed[i],"soft reset discarded homed flag");
  }
  check(Maslow.error && Maslow.watchdogFired && Maslow.errorMessage=="latched stop","soft reset cleared safety latch");
  check(Maslow.lastCallToUpdate==now,"soft reset did not refresh update watchdog timestamp");
 }
 std::cout<<"PASS production begin: one power-on initialization; no self-test, zero, or NVS restore on three soft resets; references and latches preserved\n";
}
'''.replace('METHOD',method).replace('DIRECTION_FLAGS',direction_flags)
        compiler=Path(os.environ.get('CXX',str(active/'.pio/host-tools/ziglang/zig.exe')))
        command=[str(compiler)]+(['c++'] if compiler.stem.lower()=='zig' else [])
        with tempfile.TemporaryDirectory() as temp:
            cpp=Path(temp)/'begin_once.cpp';exe=Path(temp)/'begin_once.exe'
            cpp.write_text(harness,encoding='utf-8')
            subprocess.run(command+['-std=c++17','-O0',str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)
            subprocess.run([str(exe),'initial-hardware-fault'],check=True)


if __name__=='__main__':unittest.main()
