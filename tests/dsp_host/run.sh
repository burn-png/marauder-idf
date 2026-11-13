#!/bin/sh
# Host-side checks of the DSP algorithms (x86, not the ESP32). Run from this folder: sh run.sh
set -e
M=../../main
g++ -std=c++17 -O2 -I$M -I$M/include -I. test_dsp.cpp $M/dsp.cpp $M/dsp_adv.cpp $M/dsp_cfg.cpp -o /tmp/dsp_test && /tmp/dsp_test
g++ -std=c++17 -O2 -I$M -I$M/include false_alarm.cpp $M/dsp.cpp $M/dsp_adv.cpp $M/dsp_cfg.cpp -o /tmp/dsp_fa && /tmp/dsp_fa
g++ -std=c++17 -O2 -I$M -I$M/include bench.cpp $M/dsp.cpp $M/dsp_adv.cpp $M/dsp_cfg.cpp -o /tmp/dsp_bench && /tmp/dsp_bench
