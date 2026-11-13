#!/bin/sh
cd "$(dirname "$0")" && g++ -std=c++17 -g -fsanitize=address,undefined -Wall -I../nav_host/stub -I../../main -o /tmp/cdraw_test test_cdraw.cpp ../../main/osm.cpp && /tmp/cdraw_test
