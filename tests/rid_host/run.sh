#!/bin/sh
# Host-side checks of the Remote ID adapter + Marauder decoder. Run from this folder: sh run.sh
set -e
M=../../main
g++ -std=c++17 -O1 -Wall -DRID_HOST -I../nav_host/stub -I$M -I$M/include -I$M/rid test_rid.cpp $M/rid.cpp $M/rid/RemoteIdDecoder.cpp $M/rid/RemoteIdModel.cpp -o /tmp/rid_test && /tmp/rid_test
