"""Execute production program-flow handling after normal and aborted buffer sync."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest


class ProgramFlowAbortTest(unittest.TestCase):
    def test_aborted_program_does_not_report_completion_or_run_hooks(self):
        active = Path(__file__).resolve().parents[3]
        root = Path(os.environ.get('GCODE_SOURCE_ROOT', active))
        source = (root/'FluidNC/src/GCode.cpp').read_text(encoding='utf-8-sig')
        start = source.index('gc_state.modal.program_flow = gc_block.modal.program_flow;')
        end = source.index('    // TODO: %', start)
        body = source[start:end]
        harness = r'''
#include <cstdlib>
#include <iostream>
#include <map>
enum class Error { Ok, SystemGcLock };
enum class State { Idle, CheckMode, Alarm };
enum class ProgramFlow { Running, OptionalStop, Paused, CompletedM2, CompletedM30 };
enum class Motion { Linear };
enum class Plane { XY };
enum class Distance { Absolute };
enum class FeedRate { UnitsPerMin };
enum class CoordIndex { G54 };
enum class SpindleState { Disable };
enum class Override { Disabled, ParkingMotion };
enum class FeedOverride { Default };
enum class RapidOverride { Default };
enum class SpindleSpeedOverride { Default };
enum class Message { ProgramEnd };
struct Modal {
    ProgramFlow program_flow=ProgramFlow::Running;
    Motion motion{}; Plane plane_select{}; Distance distance{}; FeedRate feed_rate{};
    CoordIndex coord_select{}; SpindleState spindle{}; int coolant=0; Override override{};
};
struct { Modal modal; float coord_system[6]{}; } gc_state;
struct { Modal modal; } gc_block;
struct {
    bool aborted=false; State current=State::Idle;
    bool abort() { return aborted; }
    State state() { return current; }
    void set_f_override(FeedOverride) {}
    void set_r_override(RapidOverride) {}
    void set_spindle_speed_ovr(SpindleSpeedOverride) {}
} sys;
int syncs=0, reports=0, hooks=0, holds=0, realtime=0, io_stops=0;
bool abort_sync=false;
void protocol_buffer_synchronize() {
    ++syncs;
    if(abort_sync) { sys.aborted=true; sys.current=State::Alarm; }
}
int feedHoldEvent;
void protocol_send_event(int*) { ++holds; }
void protocol_execute_realtime() { ++realtime; }
struct Start { bool _deactivateParking=false; } startup;
struct Coolant { void off() { ++io_stops; } } coolant;
struct Config { bool _enableParkingOverrideControl=false; Start* _start=&startup; Coolant* _coolant=&coolant; } configuration;
auto config=&configuration;
struct Spindle { void spinDown() { ++io_stops; } } spindle_object;
auto spindle=&spindle_object;
struct Coord { void get(float*) {} } coordinate;
std::map<CoordIndex,Coord*> coords{{CoordIndex::G54,&coordinate}};
constexpr bool RESTORE_OVERRIDES_AFTER_PROGRAM_END=true;
int report_ovr_counter=0;
void gc_wco_changed() {}
void report_feedback_message(Message) { ++reports; }
void user_m30() { ++hooks; }
Error execute_program_flow() {
PRODUCTION_BODY
    return Error::Ok;
}
void check(bool condition,const char* message) {
    if(!condition) { std::cerr<<"FAIL "<<message<<"\n";std::exit(2); }
}
void reset(ProgramFlow flow,bool cancel) {
    gc_state={};gc_block={};gc_block.modal.program_flow=flow;
    sys.aborted=false;sys.current=State::Idle;abort_sync=cancel;
    syncs=reports=hooks=holds=realtime=io_stops=0;
}
int main() {
    for(auto flow:{ProgramFlow::CompletedM2,ProgramFlow::CompletedM30,ProgramFlow::Paused}) {
        reset(flow,true);
        check(execute_program_flow()==Error::SystemGcLock,"aborted sync returned success");
        check(syncs==1 && sys.aborted && sys.current==State::Alarm,"abort state changed");
        check(reports==0 && hooks==0 && holds==0 && realtime==0 && io_stops==0,
              "aborted flow reported completion, paused, or ran end actions");
    }
    for(auto flow:{ProgramFlow::CompletedM2,ProgramFlow::CompletedM30}) {
        reset(flow,false);
        check(execute_program_flow()==Error::Ok,"normal completion rejected");
        check(syncs==1 && reports==1 && hooks==1 && io_stops==2,"normal end actions changed");
        check(gc_state.modal.program_flow==ProgramFlow::Running,"normal modal reset changed");
    }
    reset(ProgramFlow::Paused,false);
    check(execute_program_flow()==Error::Ok && holds==1 && realtime==1 && reports==0 && hooks==0,
          "normal pause changed");
    for(auto flow:{ProgramFlow::Running,ProgramFlow::OptionalStop}) {
        reset(flow,false);check(execute_program_flow()==Error::Ok && syncs==0 && reports==0 && hooks==0,
                               "running/optional flow changed");
    }
    std::cout<<"PASS production program flow: abort blocks M0/M2/M30 completion and hooks; normal end/pause unchanged\n";
}
'''.replace('PRODUCTION_BODY', body)
        compiler=Path(os.environ.get('CXX', str(active/'.pio/host-tools/ziglang/zig.exe')))
        command=[str(compiler)]+(['c++'] if compiler.stem.lower()=='zig' else [])
        with tempfile.TemporaryDirectory() as directory:
            cpp=Path(directory)/'program_flow.cpp';exe=Path(directory)/'program_flow.exe'
            cpp.write_text(harness,encoding='utf-8')
            subprocess.run(command+['-std=c++17','-O0',str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)


if __name__=='__main__':
    unittest.main()
