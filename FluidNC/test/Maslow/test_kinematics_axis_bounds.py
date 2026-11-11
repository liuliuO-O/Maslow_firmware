"""Run production Maslow transforms on five/six-axis buffers and segmented moves."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest


def extract(source, signature):
    start = source.index(signature)
    begin = source.index('{', start)
    depth, end = 1, begin + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class MaslowAxisBoundsTest(unittest.TestCase):
    def test_five_and_six_axis_transforms(self):
        root = Path(os.environ.get('MASLOW_SOURCE_ROOT', Path(__file__).resolve().parents[3]))
        source = (root/'FluidNC/src/Kinematics/MaslowKinematics.cpp').read_text(encoding='utf-8-sig')
        methods = '\n'.join(extract(source, name) for name in (
            'bool MaslowKinematics::cartesian_to_motors(',
            'void MaslowKinematics::motors_to_cartesian(',
            'void MaslowKinematics::transform_cartesian_to_motors(',
        ))
        harness = r'''
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>
#include "SavedAnchorRecovery.h"
constexpr int MAX_N_AXIS=6, X_AXIS=0, Y_AXIS=1, Z_AXIS=2;
constexpr int _TL=0, _TR=1, Coord_Z=2, READY_TO_CUT=7, ARM_COUNT=4;
#define log_error(...) ((void)0)
#define log_info(...) ((void)0)
struct Axes { size_t _numberAxis=5; } axes;
struct { Axes* _axes=&axes; } configuration;
auto config=&configuration;
struct {
  float workAreaX=500, workAreaY=300, workAreaCenterOffsetX=0, workAreaCenterOffsetY=0;
  float scaleX=2, scaleY=3, jogMaxFeed=500;
  struct { int currentState=READY_TO_CUT; } calibration;
} Maslow;
struct plan_line_data_t { struct { bool inverseTime=false, rapidMotion=false; } motion; float feed_rate=120; bool is_jog=false; };
float vector_distance(float* a,float* b,int n) { float s=0; for(int i=0;i<n;++i)s+=(a[i]-b[i])*(a[i]-b[i]); return std::sqrt(s); }
int get_axis_motor_steps(int axis) { return 100+axis; }
float steps_to_mpos(int steps,int) { return float(steps); }
std::vector<std::vector<float>> submitted;
std::vector<plan_line_data_t> plans;
bool cancelSubmission=false;
bool mc_move_motors(float* motors,plan_line_data_t* plan) {
  if (cancelSubmission) return false;
  plans.push_back(*plan);
  submitted.emplace_back(motors,motors+axes._numberAxis);
  return true;
}
bool mc_linear(float*,plan_line_data_t*,float*);
class MaslowKinematics {
public:
  bool _isSegmenting=false, _fixedZ=false;
  float _maxSegmentLength=5;
  float _spoilboardThickness=0,_workThickness=0;
  float anchor_location[4][3]{};
  bool cartesian_to_motors(float*,plan_line_data_t*,float*);
  void transform_cartesian_to_motors(float*,float*);
  void motors_to_cartesian(float*,float*,int);
  float computeTL(float x,float,float) { return 1000+x; }
  float computeTR(float,float y,float) { return 1100+y; }
  float computeBL(float x,float y,float) { return 1200+x+y; }
  float computeBR(float x,float y,float) { return 1300-x-y; }
  float compute(int arm,float x,float y,float z) {
    switch(arm) { case 0:return computeTL(x,y,z); case 1:return computeTR(x,y,z);
                  case 2:return computeBL(x,y,z); default:return computeBR(x,y,z); }
  }
  float measurementToXYPlane(float value,float) { return value; }
  bool computeXYfromBeltLengths(float a,float b,float& x,float& y) {
    if (a>900 && b>1000) { x=a-1000; y=b-1100; }
    else { x=a; y=b; }
    return true;
  }
} kin;
METHODS
bool mc_linear(float* target,plan_line_data_t* data,float* position) { return kin.cartesian_to_motors(target,data,position); }
void check(bool ok,const char* message) { if(!ok) { std::cerr<<"FAIL "<<message<<"\n"; std::exit(2); } }
int main() {
  constexpr float guard=918273.0f;
  // A long XY jog keeps the cap across every segment. G-code and Z-only jogs keep their feed.
  float position0[MAX_N_AXIS]={}, jog[MAX_N_AXIS]={30,0,0}, zJog[MAX_N_AXIS]={0,0,5};
  plan_line_data_t fast;
  fast.feed_rate=2500;
  fast.is_jog=true;
  check(kin.cartesian_to_motors(jog,&fast,position0),"capped jog failed");
  check(plans.size()==6 && fast.feed_rate==2500,"jog cap modified input or segment count");
  for (auto& block : plans) check(std::fabs(block.feed_rate-1000*std::sqrt(3.0))<.01,"XY jog segment exceeded cap");
  plans.clear();
  fast.feed_rate=120;
  check(kin.cartesian_to_motors(jog,&fast,position0),"low-speed jog failed");
  for (auto& block : plans) check(std::fabs(block.feed_rate-240*std::sqrt(3.0))<.01,"XY jog below cap was changed");
  plans.clear();
  fast.feed_rate=2500;
  check(kin.cartesian_to_motors(zJog,&fast,position0) && plans[0].feed_rate==2500,"Z-only jog was capped");
  plans.clear();
  fast.is_jog=false;
  check(kin.cartesian_to_motors(jog,&fast,position0),"uncapped G-code move failed");
  for (auto& block : plans) check(std::fabs(block.feed_rate-5000*std::sqrt(3.0))<.01,"program feed was capped");
  plans.clear();
  for(size_t count:{5u,6u}) {
    axes._numberAxis=count;
    for(int state:{READY_TO_CUT,0}) {
      Maslow.calibration.currentState=state;
      float cartesian[MAX_N_AXIS]={2,3,4,0,0,0};
      std::vector<float> motor(count+1,guard);
      kin.transform_cartesian_to_motors(motor.data(),cartesian);
      check(motor[count]==guard,"transform wrote beyond configured axis count");
      check(motor[4]==4,"Z passthrough changed");
      check(motor[0]==(state==READY_TO_CUT?1004:100),"TL mapping/hold changed");
      check(motor[1]==(state==READY_TO_CUT?1109:101),"TR mapping/hold changed");
      if(count==6)check(motor[5]==0,"legacy sixth motor is not zero");
    }
    float motors[MAX_N_AXIS]={200,300,400,500,6,0};
    std::vector<float> coordinates(count+1,guard);
    kin.motors_to_cartesian(coordinates.data(),motors,int(count));
    check(coordinates[count]==guard,"forward transform overwrote guard");
    check(coordinates[0]==100 && coordinates[1]==100 && coordinates[2]==6,"XYZ forward mapping changed");
    for(size_t i=3;i<count;++i)check(coordinates[i]==0,"unused cartesian axis is uninitialized");
    Maslow.calibration.currentState=READY_TO_CUT;
    float pose[MAX_N_AXIS]={45,-32,6,0,0,0};
    kin.transform_cartesian_to_motors(motors,pose);
    motors[0]+=1;
    kin.motors_to_cartesian(coordinates.data(),motors,int(count));
    check(std::fabs(coordinates[0]-45.3)<.002 && std::fabs(coordinates[1]+32.13333)<.002,
          "Ready pose did not use the lower belts");
    plan_line_data_t data;
    float position[MAX_N_AXIS]={0,0,0,0,0,0};
    float short_move[MAX_N_AXIS]={5,0,0,0,0,0};
    submitted.clear();
    check(kin.cartesian_to_motors(short_move,&data,position),"5mm move failed");
    check(submitted.size()==1 && submitted[0][0]==1010,"5mm motor mapping changed");
    float long_move[MAX_N_AXIS]={30,0,0,0,0,0};
    submitted.clear();
    check(kin.cartesian_to_motors(long_move,&data,position),"segmented move failed");
    check(submitted.size()==6 && submitted.back()[0]==1060,"segmentation output incorrect");
    check(!kin._isSegmenting,"segmentation flag not restored");
    check(position[0]==0 && data.feed_rate==120,"queueing changed caller position or feed");
    for (auto& plan : plans) {
      check(std::fabs(plan.feed_rate - 240*std::sqrt(3.0))<0.01,"motor and Cartesian duration differ");
    }
    plans.clear();
    data.motion.inverseTime=true;
    check(kin.cartesian_to_motors(long_move,&data,position),"inverse-time move failed");
    check(plans.size()==6,"wrong inverse-time segment count");
    for (auto& plan : plans) {
      check(plan.motion.inverseTime && plan.feed_rate==720,"inverse time not distributed across segments");
    }
    data.motion.inverseTime=false;
    cancelSubmission=true;
    check(!kin.cartesian_to_motors(long_move,&data,position),"cancelled segment was accepted");
    check(!kin._isSegmenting && position[0]==0,"cancel left stale segmentation state");
    cancelSubmission=false;
    plans.clear();
  }
  for(size_t invalid:{4u,7u}) {
    axes._numberAxis=invalid;
    plan_line_data_t data;
    float target[MAX_N_AXIS]={5,0,0,0,0,0};
    float position[MAX_N_AXIS]={0,0,0,0,0,0};
    submitted.clear();
    check(!kin.cartesian_to_motors(target,&data,position),"invalid axis count was accepted");
    check(submitted.empty(),"invalid axis count submitted motion");
  }
  std::cout<<"PASS production transforms: 5/6-axis canaries, ready/unready, XYZ/unused axes, 5mm and segmented30mm, invalid counts rejected\n";
}
'''.replace('METHODS', methods)
        compiler = Path(os.environ.get('CXX', str(root/'.pio/host-tools/ziglang/zig.exe')))
        command = [str(compiler)] + (['c++'] if compiler.stem.lower() == 'zig' else [])
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory)/'bounds.cpp'
            exe = Path(directory)/'bounds.exe'
            cpp.write_text(harness, encoding='utf-8')
            subprocess.run(command+['-std=c++17','-O0','-I',str(root/'FluidNC/src/Maslow'),str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__ == '__main__':
    unittest.main()
