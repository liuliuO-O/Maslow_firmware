"""Run extracted production stop/reset and late-event code; no hardware access."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest


def extract(source, signature):
    start=source.index(signature)
    begin=source.index('{',start)
    depth,end=1,begin+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}')
        end+=1
    return source[start:end]


class MaslowStopAbortTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        active=Path(os.environ.get('MASLOW_SOURCE_ROOT',Path(__file__).resolve().parents[3]))
        root=Path(os.environ.get('STOP_SOURCE_ROOT',str(active)))
        sources={name:(root/'FluidNC/src'/name).read_text(encoding='utf-8-sig') for name in
                 ('Maslow/Maslow.cpp','MotionControl.cpp','Protocol.cpp','ProcessSettings.cpp')}
        production='\n'.join(extract(sources[file],signature) for file,signature in [
            ('MotionControl.cpp','void mc_reset()'),
            ('Maslow/Maslow.cpp','static void stopEverything()'),
            ('Maslow/Maslow.cpp','void Maslow_::panic()'),
            ('Maslow/Maslow.cpp','void Maslow_::eStop(String message)'),
            ('ProcessSettings.cpp','static Error maslow_estop('),
            ('Protocol.cpp','void protocol_do_cycle_stop()'),
            ('Protocol.cpp','static void protocol_do_initiate_cycle() {'),
            ('Protocol.cpp','static void protocol_do_auto_cycle_start()')])
        harness=r'''
#include <atomic>
#include <iostream>
#include <stdexcept>
#include <string>
using String=std::string;
#define log_error(...) ((void)0)
#define log_warn(...) ((void)0)
#define log_info(...) ((void)0)
enum class State {Idle,Cycle,Jog,Homing,Hold,SafetyDoor,Sleep,Alarm,ConfigAlarm,CheckMode};
enum class ExecAlarm {None,HardLimit,SoftLimit,HomingFailReset,AbortCycle};
enum class Error {Ok};
namespace WebUI {enum class AuthenticationLevel {LEVEL_GUEST};}
struct Channel {};
struct Suspend {unsigned value=0;struct {bool jogCancel=false,safetyDoorAjar=false,holdComplete=false,motionCancel=false;} bit;};
struct System {
 State current=State::Idle;
 struct {bool executeHold=false,executeSysMotion=false;} step_control;
 Suspend suspended;
 State state() const{return current;}
 void set_state(State value){current=value;}
 Suspend suspend() const{return suspended;}
 void set_suspend(Suspend value){suspended=value;}
} sys;
volatile bool rtReset=false;
ExecAlarm rtAlarm=ExecAlarm::None;
std::atomic<bool> feedHoldPending{false};
bool soft_limit=false,pwm=false,awake=false,pulse_asserted=false;
int unsteps=0,idles=0,cleanups=0,wakeups=0,blocks=3;
struct plan_block_t {bool is_jog=false;} block;
plan_block_t* plan_get_current_block(){return blocks?&block:nullptr;}
void plan_reset(){blocks=0;}
void plan_cycle_reinitialize(){}
void gc_sync_position(){}
void plan_sync_position(){}
void protocol_disable_steppers(){}
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool inMotionState(){return sys.state()==State::Cycle||sys.state()==State::Jog||sys.state()==State::Homing;}
namespace Stepper {
 // Real stop_stepping unsteps pins; only go_idle disables the awake ISR.
 void stop_stepping(){++unsteps;pulse_asserted=false;}
 void go_idle(){++idles;awake=false;stop_stepping();protocol_disable_steppers();}
 void reset(){awake=false;pulse_asserted=false;}
 void prep_buffer(){}
 void wake_up(){++wakeups;awake=true;}
}
namespace Machine {struct Homing {static void cycleStop(){}};}
struct Maslow_ {
 bool error=false,watchdogFired=false;
 int panicCount=0;
 String errorMessage;
 void stopMotors(){pwm=false;}
 void stop(){
  stopMotors();  // Production stop() also disables PWM before stopJob.
  // Entry to the potentially blocking stopJob cleanup must already be safe.
  check(!pwm,"PWM still on at blocking cleanup");
  check(rtReset,"protocol abort not requested before cleanup");
  check(!awake,"step ISR still awake at blocking cleanup");
  ++cleanups;
 }
 void panic();
 void eStop(String message="Emergency stop triggered.");
} Maslow;
void protocol_do_feedhold(){sys.set_state(State::Hold);}
PRODUCTION
void prepare(State state){
 sys=System{};sys.set_state(state);rtReset=false;rtAlarm=ExecAlarm::None;
 feedHoldPending.store(false);soft_limit=false;pwm=true;awake=inMotionState();pulse_asserted=awake;
 unsteps=idles=cleanups=wakeups=0;blocks=3;block.is_jog=(state==State::Jog);
 Maslow.error=false;Maslow.watchdogFired=false;Maslow.errorMessage.clear();
}
void entrypoints(){
 Channel channel;
 for(State state:{State::Cycle,State::Jog,State::Homing,State::Idle}){
  for(int entry=0;entry<3;++entry){
   prepare(state);
   if(entry==0)Maslow.eStop("tracking fault");
   else if(entry==1)Maslow.panic();
   else check(maslow_estop(nullptr,WebUI::AuthenticationLevel::LEVEL_GUEST,channel)==Error::Ok,"estop command failed");
   check(sys.state()==State::Alarm&&!pwm&&!awake&&rtReset,"stop did not latch Alarm, abort and outputs off");
   check(cleanups>0,"cleanup omitted");
   if(state!=State::Idle)check(unsteps>0,"original motion state hidden from mc_reset");
   if(state==State::Homing)check(rtAlarm==ExecAlarm::HomingFailReset,"wrong homing abort alarm");
   else if(state!=State::Idle)check(rtAlarm==ExecAlarm::AbortCycle,"wrong cycle abort alarm");
   if(entry!=1)check(Maslow.error,"emergency error not latched");
   Maslow.eStop("repeat");
   check(rtReset&&sys.state()==State::Alarm&&!awake&&!pwm,"repeat stop restarted motion or cleared abort/alarm");
  }
 }
}
void completion(){
 for(State state:{State::Alarm,State::ConfigAlarm}){
  prepare(state);pwm=false;awake=false;
  protocol_do_cycle_stop();
  check(sys.state()==state,"late cycleStop cleared latched alarm");
  protocol_do_auto_cycle_start();
  check(wakeups==0&&sys.state()==state,"late event restarted old planner blocks");
 }
 prepare(State::Cycle);protocol_do_cycle_stop();
 check(sys.state()==State::Idle,"normal cycle no longer ends Idle");
 prepare(State::Hold);sys.step_control.executeHold=true;protocol_do_cycle_stop();
 check(sys.state()==State::Hold&&sys.suspend().bit.holdComplete,"completed hold no longer awaits explicit resume");
 prepare(State::Hold);sys.suspended.bit.jogCancel=true;protocol_do_cycle_stop();
 check(sys.state()==State::Idle&&blocks==0,"jog cancellation no longer clears plan");
}
void queued_events(){
 prepare(State::Cycle);Maslow.eStop("tracking fault");
 // Events queued near the fault can be processed before/after the outer reset.
 protocol_do_cycle_stop();protocol_do_auto_cycle_start();
 check(sys.state()==State::Alarm&&wakeups==0&&!awake,"queued events revived cancelled motion before reset");
 // Model Main::reset_variables clearing planner and stepper after protocol abort.
 plan_reset();Stepper::reset();rtReset=false;
 protocol_do_cycle_stop();protocol_do_auto_cycle_start();
 check(sys.state()==State::Alarm&&wakeups==0&&blocks==0,"stale events cleared Alarm after outer reset");
}
int main(int argc,char** argv){
 try{
  check(argc==2,"missing scenario");std::string scenario=argv[1];
  if(scenario=="entrypoints")entrypoints();
  else if(scenario=="completion")completion();
  else if(scenario=="queued")queued_events();
  else check(false,"unknown scenario");
  std::cout<<"PASS production stop/abort "<<scenario<<"\n";
 }catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<"\n";return 2;}
}
'''.replace('PRODUCTION',production)
        cls.directory=tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        cpp=Path(cls.directory.name)/'stop_abort.cpp'
        cls.executable=Path(cls.directory.name)/'stop_abort.exe'
        cpp.write_text(harness,encoding='utf-8')
        compiler=Path(os.environ.get('CXX',str(active/'.pio/host-tools/ziglang/zig.exe')))
        command=[str(compiler)]+(['c++'] if compiler.stem.lower()=='zig' else [])
        subprocess.run(command+['-std=c++17','-O0',str(cpp),'-o',str(cls.executable)],check=True)

    def test_stop_aborts_outputs_before_blocking_cleanup(self):
        subprocess.run([str(self.executable),'entrypoints'],check=True)

    def test_late_completion_preserves_alarm_and_normal_hold(self):
        subprocess.run([str(self.executable),'completion'],check=True)

    def test_queued_events_cannot_restart_cancelled_motion(self):
        subprocess.run([str(self.executable),'queued'],check=True)


if __name__=='__main__':
    unittest.main()
