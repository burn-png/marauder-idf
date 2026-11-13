#!/bin/sh
cd "$(dirname "$0")" && python3 ../../tools/make_snake.py /tmp/SNAKE_TEST.APP >/dev/null && g++ -std=c++17 -g -fsanitize=address,undefined -Wall -I../../main -o /tmp/appvm_test test_appvm.cpp ../../main/appvm.cpp -include ctype.h && /tmp/appvm_test /tmp/SNAKE_TEST.APP
