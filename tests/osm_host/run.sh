#!/bin/sh
# Host checks of the OSM streaming indexer + cell renderer on synthetic maps. Run from this folder: sh run.sh
set -e
M=../../main
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -Wall -I$M -I$M/include test_osm.cpp $M/osm.cpp -o /tmp/osm_test && /tmp/osm_test
