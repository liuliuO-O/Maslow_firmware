"""Execute the production C++ LM solver on deterministic belt measurements.

This verifies math/quality gates on a host; it does not certify real motor motion.
Set CXX to a compiler, or install Zig under the project's .pio/host-tools.
"""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest


def extract_method(source, signature):
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class CalibrationFitTest(unittest.TestCase):
    def test_production_fit_and_quality_gates(self):
        root = Path(os.environ.get('MASLOW_SOURCE_ROOT', Path(__file__).resolve().parents[3]))
        maslow = root / 'FluidNC/src/Maslow'
        source = (maslow / 'Calibration.cpp').read_text(encoding='utf-8-sig')
        helpers = source[source.index('    struct CalibrationMeasurement'):source.index('}  // namespace')]
        methods = '\n'.join(extract_method(source, signature) for signature in (
            'bool Calibration::recomputeAnchorsWithLevenbergMarquardt(',
            'void Calibration::allocateCalibrationMemory(',
            'void Calibration::deallocateCalibrationMemory(',
        ))
        harness = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#define private public
#include "Calibration.h"
#undef private
#define CALIBRATION_GRID_SIZE_MAX 102
unsigned long millis() { return 0; }
std::vector<std::string> logs;
#define log_info(x) do { std::ostringstream s; s << x; logs.push_back(s.str()); } while(0)
#define log_error(x) log_info(x)
#define log_debug(x) log_info(x)
void serviceCalibrationWatchdogs(bool = false) {}
namespace {
HELPERS
}
struct KinematicsStub {
    double tlX=40, tlY=1000, trX=1100, trY=1040, brX=1050;
    int applied=0;
    double getTlX() { return tlX; }
    double getTlY() { return tlY; }
    double getTrX() { return trX; }
    double getTrY() { return trY; }
    double getBrX() { return brX; }
    double getCenterX() { return (tlX+trX+brX)/4; }
    double getCenterY() { return (tlY+trY)/4; }
    void setCalibrationAnchors(double a,double b,double c,double d,double e) {
        tlX=a; tlY=b; trX=c; trY=d; brX=e; ++applied;
    }
} kin;
KinematicsStub* getKinematics() { return &kin; }
struct { double x=0, y=0; } Maslow;
Calibration::Calibration() {}
METHODS
void fill(Calibration& c, double noise, bool perturb=false) {
    kin=KinematicsStub{};
    c.allocateCalibrationMemory();
    std::mt19937 rng(2468);
    std::uniform_real_distribution<double> random(-noise, noise);
    const double ax[]={40,1100,0,1050}, ay[]={1000,1040,0,0};
    const double xy[17][2]={
        {0,0},{150,0},{150,50},{0,50},{-150,50},{-150,0},
        {0,0},{0,-50},{-150,-50},{-150,0},{-150,50},
        {0,50},{150,50},{150,0},{150,-50},{0,-50},{0,0}
    };
    for(int i=0;i<17;++i) for(int a=0;a<4;++a) {
        const double x=520+xy[i][0], y=500+xy[i][1];
        c.calibration_data[i][a]=std::hypot(x-ax[a],y-ay[a])+random(rng);
    }
    if(perturb) { kin.tlX-=20; kin.tlY+=15; kin.trX+=25; kin.trY-=10; kin.brX+=20; }
    logs.clear();
}
void report(const char* name, bool result) {
    std::cout << name << " result=" << result << '\n';
    for(const auto& log:logs) if(log.find("fit:")!=std::string::npos || log.find("fit failed")!=std::string::npos) std::cout<<log<<'\n';
}
int main() {
    Calibration c;
    fill(c,0);
    assert(c.recomputeAnchorsWithLevenbergMarquardt(17));
    assert(c.lastRecomputePassed && kin.applied==1);
    assert(std::abs(kin.tlX-40)<0.1 && std::abs(kin.brX-1050)<0.1);
    assert(std::isfinite(Maslow.x) && std::isfinite(Maslow.y));
    assert(std::abs(c.calibrationGrid[16][0]-Maslow.x)<1e-3);
    assert(std::abs(c.calibrationGrid[16][1]-Maslow.y)<1e-3);
    report("exact",true);
    for(int count:{6,15,17}) {
        fill(c,0,true);
        bool ok=c.recomputeAnchorsWithLevenbergMarquardt(count);
        report((std::string("perturbed-")+std::to_string(count)).c_str(),ok);
        assert(ok && kin.applied==1);
    }
    fill(c,20,true);
    c.calibrationBootstrap17=true;
    bool loose=c.recomputeAnchorsWithLevenbergMarquardt(17);
    report("noise20-bootstrap",loose);
    assert(loose && kin.applied==1);
    fill(c,20,true);
    c.calibrationBootstrap17=false;
    bool strict=c.recomputeAnchorsWithLevenbergMarquardt(17);
    report("noise20-precision",strict);
    assert(!strict && !c.lastRecomputePassed && kin.applied==0);
    fill(c,100,true);
    c.calibrationBootstrap17=true;
    assert(!c.recomputeAnchorsWithLevenbergMarquardt(17));
    assert(!c.lastRecomputePassed && kin.applied==0);
    report("noise100-rejected",false);
    for(float bad:{0.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        fill(c,0);
        c.lastRecomputePassed=true;
        c.calibration_data[8][2]=bad;
        assert(!c.recomputeAnchorsWithLevenbergMarquardt(17));
        assert(!c.lastRecomputePassed && kin.applied==0);
    }
    assert(!c.recomputeAnchorsWithLevenbergMarquardt(0));
    c.deallocateCalibrationMemory();
    c.deallocateCalibrationMemory();
    std::cout << "PASS production LM: exact, 6/15/17, relaxed-vs-strict, outlier, invalid input, frame rebase\n";
}
'''.replace('HELPERS', helpers).replace('METHODS', methods)
        zig = root / '.pio/host-tools/ziglang/zig.exe'
        compiler = Path(os.environ.get('CXX', str(zig)))
        command = [str(compiler)] + (['c++'] if compiler.stem.lower() == 'zig' else [])
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / 'Arduino.h').write_text('#pragma once\n#include <cstdint>\nunsigned long millis();\n')
            cpp, exe = tmp / 'fit.cpp', tmp / 'fit.exe'
            cpp.write_text(harness, encoding='utf-8')
            subprocess.run(command + ['-std=c++17', '-O0', '-I', str(tmp), '-I', str(maslow), str(cpp), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
