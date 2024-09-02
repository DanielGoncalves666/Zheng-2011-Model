#!/bin/bash

gcc -O2 -o build/zheng.exe src/*.c -lm -g && ./build/zheng.exe "$@"