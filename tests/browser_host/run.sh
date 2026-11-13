#!/bin/sh
cd "$(dirname "$0")" && g++ -std=c++17 -g -fsanitize=address,undefined -Wall -Istub -I../../main -o /tmp/browser_test test_browser.cpp ../../main/browser.cpp && /tmp/browser_test
