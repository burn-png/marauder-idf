#!/bin/sh
# Host-side checks of the geodesy + navigation engine. Run from this folder: sh run.sh
set -e
M=../../main
g++ -std=c++17 -O2 -Wall -DNAV_HOST -Istub -I$M -I$M/include test_nav.cpp $M/nav.cpp -o /tmp/nav_test && /tmp/nav_test
