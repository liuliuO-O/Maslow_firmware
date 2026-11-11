"""Compile the production AS5600 driver and exercise real feedback publication.

Only Arduino/I2C/PWM are replaced. No device connection or motor motion occurs.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
from test_calibration_bootstrap import compiler_command, extract_function

ARDUINO = r'''
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
using String = std::string;
using std::abs;
using std::round;
constexpr int OUTPUT=1, LOW=0;
constexpr double PI=3.141592653589793;
#define F(x) x
extern uint32_t clockUs;
inline uint32_t micros(){return clockUs;}
inline unsigned long millis(){return clockUs/1000;}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline int digitalRead(int){return 0;}
'''
WIRE = r'''
#pragma once
#include "Arduino.h"
#include <deque>
class TwoWire {
public:
 std::deque<int> angles;
 int value=0, index=0;
 void beginTransmission(uint8_t){}
 void write(uint8_t){}
 uint8_t endTransmission(){return 0;}
 uint8_t requestFrom(uint8_t,uint8_t count){
   index=0;
   if(count==1){value=0x20;return 1;}
   if(angles.empty()) return 0;
   value=angles.front();angles.pop_front();
   return value<0?0:2;
 }
 int read(){return index++==0?(value>>8)&255:value&255;}
};
extern TwoWire Wire;
'''

HARNESS = r'''
#include <cassert>
#include <iostream>
#include <limits>
#include "Arduino.h"
#include "Wire.h"
#include "EncoderPosition.h"
#define private public
#include "MotorUnit.h"
#undef private
uint32_t clockUs=0;
TwoWire Wire;
DCMotor::DCMotor(){}
MiniPID::MiniPID(){}
bool muxOk=true;
int muxCalls=0;
bool QWIICMUX::setPort(uint8_t){++muxCalls;return muxOk;}
struct Machine {QWIICMUX I2CMux;String axis_id_to_label(int){return "axis";}} Maslow;
#define log_warn(...) ((void)0)
METHODS

void sample(AS5600& sensor,int angle,uint32_t elapsed,int expected,int error=0){
 clockUs+=elapsed; Wire.angles={angle};
 assert(sensor.getCumulativePosition()==expected);
 assert(sensor.lastError()==error);
 assert(Wire.angles.empty());
}
int main(){
 AS5600 sensor;
 sensor.setPositionRateLimit(93149); // ceil(1000 mm/s * 4096 / 43.9736).
 Wire.angles={4090};sensor.resetCumulativePosition(123);
 assert(sensor.lastError()==0);
 sample(sensor,10,4000,139); // Forward crossing, 16 counts.
 sample(sensor,4090,4000,123); // Reverse crossing.
 sample(sensor,4000,4000,33);
 sample(sensor,1000,4000,33,AS5600_ERROR_POSITION_JUMP);
 sample(sensor,3990,100,23); // Rejected angle did not poison the baseline.
 sample(sensor,-1,4000,23,AS5600_ERROR_I2C_READ_3);
 sample(sensor,3980,100,13);
 // A failed zero must preserve the accepted angle and cumulative reference.
 Wire.angles={-1};sensor.resetCumulativePosition(9999);
 assert(sensor.lastError()==AS5600_ERROR_I2C_READ_3);
 sample(sensor,3970,4000,3);
 sample(sensor,3960,MaslowEncoder::maxSampleAgeUs+1,3,AS5600_ERROR_POSITION_STALE);
 sample(sensor,3950,4000,3,AS5600_ERROR_POSITION_STALE);
 Wire.angles={3950};sensor.resetCumulativePosition(500);
 assert(sensor.lastError()==0);sample(sensor,3940,4000,490);
 // Counter wrap must not be mistaken for stale feedback.
 clockUs=UINT32_MAX-1000;Wire.angles={100};sensor.resetCumulativePosition(0);
 sample(sensor,110,500,10);sample(sensor,120,1500,20);
 // Software reversal retains the saved ANGLE convention across zero.
 sensor.setDirection(AS5600_COUNTERCLOCK_WISE);
 Wire.angles={4090};sensor.resetCumulativePosition(0);
 sample(sensor,10,4000,-16);sample(sensor,4090,4000,0);
 // Original stationary mirror sequence must NEVER alter accumulated position.
 sensor.setDirection(AS5600_CLOCK_WISE);
 Wire.angles={841};sensor.resetCumulativePosition(700);
 sample(sensor,841,4000,700);
 for(int i=0;i<3;++i){
  sample(sensor,3255,4000,700,AS5600_ERROR_POSITION_JUMP);
  sample(sensor,841,100,700);
 }
 assert(MaslowEncoder::plausibleDelta(1535,20000,93149));
 assert(!MaslowEncoder::plausibleDelta(1536,20000,93149));
 assert(!MaslowEncoder::plausibleDelta(700,4000,93149));

 MotorUnit axis; axis._encoderAddress=3;
 axis.setEncoderGeometry(1.9988,22);
 clockUs=100000;Wire.angles={841};assert(axis.setPosition(12.0));
 const double origin=axis.getPosition();
 clockUs+=4000;Wire.angles={3255,841}; // Single spike, bounded retry recovers.
 assert(axis.updateEncoderPosition()&&axis.encoderFeedbackValid());
 assert(axis.getPosition()==origin&&axis.getEncoderReadFailures()==1);
 clockUs+=4000;Wire.angles={3255,3255};
 assert(!axis.updateEncoderPosition()&&!axis.encoderFeedbackValid());
 assert(axis.getPosition()==origin&&axis.getEncoderReadFailures()==3);
 clockUs+=100;Wire.angles={841};assert(axis.updateEncoderPosition());
 uint16_t output=999;
 muxOk=false;int before=muxCalls;
 assert(!axis.getRawEncoderAngle(output)&&output==999&&muxCalls==before+2);
 assert(!axis.zero()&&axis.getPosition()==origin);
 muxOk=true;Wire.angles={-1};assert(!axis.setPosition(999));
 assert(axis.getPosition()==origin);
 clockUs+=100;Wire.angles={841};assert(axis.updateEncoderPosition());
 clockUs+=MaslowEncoder::maxSampleAgeUs+1;
 assert(!axis.encoderFeedbackValid());
 Wire.angles={841,841};assert(!axis.updateEncoderPosition());
 Wire.angles={841};assert(axis.zero()); // Explicit baseline recovery only.
 assert(axis.encoderFeedbackValid()&&axis.getPosition()==0);
 std::cout<<"PASS production AS5600 and MotorUnit: wrap, reversal, mirror rejection, bounded retry, stale latch, failed reset/mux/read, clock wrap\n";
}
'''


class EncoderFeedbackTest(unittest.TestCase):
    def test_production_driver_rejects_before_accumulation(self):
        root=Path(__file__).resolve().parents[3]
        folder=root/'FluidNC/src/Maslow'
        source=(folder/'MotorUnit.cpp').read_text(encoding='utf-8-sig')
        methods='\n'.join(extract_function(source,signature) for signature in (
            'bool MotorUnit::updateEncoderPosition(',
            'bool MotorUnit::encoderFeedbackValid(',
            'double MotorUnit::getPosition(',
            'bool MotorUnit::zero(',
            'bool MotorUnit::setPosition(',
            'bool MotorUnit::getRawEncoderAngle(',
            'void MotorUnit::setEncoderGeometry('))
        with tempfile.TemporaryDirectory() as tmp:
            tmp=Path(tmp)
            (tmp/'Arduino.h').write_text(ARDUINO,encoding='utf-8')
            (tmp/'Wire.h').write_text(WIRE,encoding='utf-8')
            (tmp/'driver').mkdir()
            (tmp/'driver/adc.h').write_text('',encoding='utf-8')
            (tmp/'esp_adc_cal.h').write_text('',encoding='utf-8')
            cpp=tmp/'feedback.cpp';exe=tmp/('feedback.exe' if os.name=='nt' else 'feedback')
            cpp.write_text(HARNESS.replace('METHODS',methods),encoding='utf-8')
            subprocess.run(compiler_command(root)+['-std=c++17','-I',str(tmp),'-I',str(folder),
                str(cpp),str(folder/'AS5600.cpp'),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_feedback_blocks_motion_and_calibration(self):
        root=Path(__file__).resolve().parents[3]
        source=(root/'FluidNC/src/Maslow/Maslow.cpp').read_text(encoding='utf-8-sig')
        start=source.index('        if (!Maslow.updateEncoderPositions()) {')
        end=source.index('\n        for (int arm',start)
        gate=source[start:end]
        scheduler=extract_function(source,'bool Maslow_::updateEncoderPositions(')
        harness=r'''
#include <cassert>
#include <cstdint>
#include <string>
using String=std::string;
uint32_t clockUs=0;
uint32_t micros(){return clockUs;}
unsigned long millis(){return clockUs/1000;}
constexpr int _TL=0,ARM_COUNT=4,TLEncoderLine=2,TREncoderLine=1,BLEncoderLine=3,BREncoderLine=0;
constexpr uint32_t ENCODER_READ_FREQUENCY_HZ=250;
#define log_error(...) ((void)0)
#define log_warn(...) ((void)0)
enum class State {Idle,Alarm,Cycle,Jog,Hold,Homing};
struct System {State value=State::Idle;State state(){return value;}} sys;
struct Axis {
 bool good=true,valid=true;int reads=0;
 bool updateEncoderPosition(){++reads;valid=good;return good;}
 bool encoderFeedbackValid(){return valid;}
};
class Maslow_ {
public:
 Axis axis[4];bool readingFromSD=false;int stops=0,faults=0,controlCalls=0;
 bool updateEncoderPositions();
 String axis_id_to_label(int){return "axis";}
 void stopMotors(){++stops;}
 void eStop(const char*){++faults;sys.value=State::Alarm;}
 void panic(){++faults;}
 void step();
} Maslow;
SCHEDULER
void Maslow_::step(){GATE ++controlCalls;}
int main(){
 Maslow.updateEncoderPositions(); // Initialize scheduler clock.
 for(auto state:{State::Cycle,State::Jog,State::Hold,State::Homing,State::Idle}){
  sys.value=state;Maslow.axis[2].good=false;clockUs+=4000;
  int oldControl=Maslow.controlCalls,oldFaults=Maslow.faults;
  Maslow.step();assert(Maslow.controlCalls==oldControl&&Maslow.stops>0);
  assert(Maslow.faults==oldFaults+(state!=State::Idle));
 }
 Maslow.axis[2].good=true;sys.value=State::Idle;clockUs+=4000;
 Maslow.step();int good=Maslow.controlCalls;
 // A skipped SD batch must not hide invalid/expired feedback.
 Maslow.readingFromSD=true;Maslow.axis[0].valid=false;clockUs+=4000;
 Maslow.step();assert(Maslow.controlCalls==good);
 Maslow.readingFromSD=false;clockUs+=4000;Maslow.step();
 assert(Maslow.controlCalls==good+1);
}
'''.replace('SCHEDULER',scheduler).replace('GATE',gate)
        with tempfile.TemporaryDirectory() as tmp:
            cpp=Path(tmp)/'gate.cpp';exe=Path(tmp)/'gate.exe'
            cpp.write_text(harness,encoding='utf-8')
            subprocess.run(compiler_command(root)+['-std=c++17',str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__=='__main__':unittest.main()
