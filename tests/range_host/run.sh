#!/bin/sh
set -e
M=../../main
g++ -std=c++17 -O1 -Wall -DRANGE_HOST -I$M -I$M/include test_range.cpp $M/rf_range.cpp -o /tmp/range_test && /tmp/range_test
