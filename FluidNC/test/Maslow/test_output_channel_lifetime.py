"""Exercise production output draining and channel retirement with delayed I/O."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest
import re


def extract(source, signature):
    start=source.index(signature)
    brace=source.index('{',start)
    depth=0
    pattern=re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]')
    for match in pattern.finditer(source,brace):
        if match.group()=='{': depth+=1
        elif match.group()=='}':
            depth-=1
            if depth==0:return source[start:match.end()]
    raise ValueError(signature)


COMMON=r'''
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <future>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
void check(bool ok,const char* message){if(!ok){std::cerr<<"FAIL "<<message<<"\n";std::exit(2);}}
'''


class OutputChannelLifetimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=Path(os.environ.get('MASLOW_SOURCE_ROOT',Path(__file__).resolve().parents[3]))

    def run_cpp(self, program):
        compiler=Path(os.environ.get('CXX',str(self.root/'.pio/host-tools/ziglang/zig.exe')))
        command=[str(compiler)]+(['c++'] if compiler.stem.lower()=='zig' else [])
        with tempfile.TemporaryDirectory() as temp:
            cpp=Path(temp)/'lifetime.cpp'
            exe=Path(temp)/'lifetime.exe'
            cpp.write_text(COMMON+program,encoding='utf-8')
            subprocess.run(command+['-std=c++17','-O0','-pthread',str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True,timeout=8)

    def test_drain_waits_for_inflight_output_not_only_empty_queue(self):
        source=(self.root/'FluidNC/src/Protocol.cpp').read_text(encoding='utf-8-sig')
        struct=source[source.index('struct LogMessage {'):source.index('};',source.index('struct LogMessage {'))+2]
        methods=extract(source,'void drain_messages()')+'\n'+extract(source,'void output_loop(')
        program=r'''
struct Print {virtual void println(const char*)=0;virtual ~Print()=default;};
STRUCT
struct Semaphore {std::mutex mutex;std::condition_variable cv;bool ready=false;};
using StaticSemaphore_t=Semaphore;
using SemaphoreHandle_t=Semaphore*;
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t* storage){return storage;}
bool xSemaphoreTake(SemaphoreHandle_t sem,int){std::unique_lock<std::mutex> lock(sem->mutex);sem->cv.wait(lock,[&]{return sem->ready;});return true;}
void xSemaphoreGive(SemaphoreHandle_t sem){{std::lock_guard<std::mutex> lock(sem->mutex);sem->ready=true;}sem->cv.notify_all();}
void vSemaphoreDelete(SemaphoreHandle_t){}
constexpr int portMAX_DELAY=-1,pdTRUE=1;
int outputTask=2;
int xTaskGetCurrentTaskHandle(){return 1;}
struct Queue {std::mutex mutex;std::deque<LogMessage> data;} queue;
auto message_queue=&queue;
bool xQueueSend(Queue* q,LogMessage* message,int){std::lock_guard<std::mutex> lock(q->mutex);q->data.push_back(*message);return true;}
bool xQueueReceive(Queue* q,LogMessage* message,int){std::lock_guard<std::mutex> lock(q->mutex);if(q->data.empty())return false;*message=q->data.front();q->data.pop_front();return true;}
size_t uxQueueMessagesWaiting(Queue* q){std::lock_guard<std::mutex> lock(q->mutex);return q->data.size();}
struct StopOutput{};
std::atomic<bool> stopOutput{false};
void vTaskDelay(int){if(stopOutput.load())throw StopOutput{};std::this_thread::sleep_for(1ms);}
METHODS
struct DelayedWriter:Print {
 std::mutex mutex;std::condition_variable cv;bool entered=false,release=false,delivered=false;
 void println(const char* text)override{
  std::unique_lock<std::mutex> lock(mutex);entered=true;cv.notify_all();
  cv.wait(lock,[&]{return release;});delivered=std::string(text)=="ok";
 }
} writer;
int main(){
 LogMessage message{&writer,(void*)"ok",false};xQueueSend(message_queue,&message,0);
 std::thread worker([]{try{output_loop(nullptr);}catch(StopOutput&){};});
 {std::unique_lock<std::mutex> lock(writer.mutex);writer.cv.wait(lock,[&]{return writer.entered;});}
 check(uxQueueMessagesWaiting(message_queue)==0,"test did not reach in-flight empty-queue case");
 auto flush=std::async(std::launch::async,[]{drain_messages();});
 bool premature=flush.wait_for(30ms)==std::future_status::ready;
 {std::lock_guard<std::mutex> lock(writer.mutex);writer.release=true;}writer.cv.notify_all();
 flush.wait();stopOutput.store(true);worker.join();
 check(!premature,"drain returned while output still referenced its channel");
 check(writer.delivered,"pending output was not delivered");
 std::cout<<"PASS output fence waits for in-flight println before releasing caller\n";
}
'''.replace('STRUCT',struct).replace('METHODS',methods)
        self.run_cpp(program)

    def test_retire_keeps_active_channel_alive_until_ack_and_output_finish(self):
        source=(self.root/'FluidNC/src/Serial.cpp').read_text(encoding='utf-8-sig')
        methods=extract(source,'void AllChannels::deregistration(')+'\n'+extract(source,'Channel* AllChannels::pollLine(')
        program=r'''
bool delivered=false;int drains=0;
struct Channel {
 bool* destroyed;bool* completed;
 Channel(bool* d,bool* c):destroyed(d),completed(c){}
 virtual Channel* pollLine(char*){return nullptr;}
 virtual ~Channel(){*destroyed=true;*completed=delivered;}
};
std::atomic<Channel*> activeChannel{nullptr};
struct KillQueue{std::deque<Channel*> data;} queue;
bool xQueueReceive(KillQueue* q,Channel** c,int){if(q->data.empty())return false;*c=q->data.front();q->data.pop_front();return true;}
void drain_messages(){++drains;delivered=true;}
class AllChannels {
public:
 std::vector<Channel*> _channelq,_retiringChannels;
 Channel* _lastChannel=nullptr;
 KillQueue* _killQueue=&queue;
 static std::mutex _mutex;
 void deregistration(Channel*);
 Channel* pollLine(char*);
};
std::mutex AllChannels::_mutex;
METHODS
int main(){
 AllChannels channels;
 bool destroyed=false,completed=false;
 Channel* active=new Channel(&destroyed,&completed);
 channels._channelq.push_back(active);channels._lastChannel=active;activeChannel.store(active);
 queue.data.push_back(active);
 channels.pollLine(nullptr);
 check(!destroyed,"channel deleted while execute_line or ack still owns it");
 check(channels._channelq.empty(),"retired channel can still be polled or broadcast to");
 activeChannel.store(nullptr);
 channels.pollLine(nullptr);
 check(destroyed && completed && drains==1,"channel freed before pending output was drained");
 delivered=false;destroyed=false;completed=false;
 Channel* duplicate=new Channel(&destroyed,&completed);
 channels._channelq.push_back(duplicate);
 queue.data.push_back(duplicate);queue.data.push_back(duplicate);
 channels.pollLine(nullptr);
 check(destroyed && completed && drains==2,"duplicate retirement was not handled once");
 std::cout<<"PASS retired channel stays alive through active command, output fence, and duplicate kill\n";
}
'''.replace('METHODS',methods)
        self.run_cpp(program)


if __name__=='__main__':
    unittest.main()
