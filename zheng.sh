#!/bin/bash

gcc -o build/zheng.exe src/*.c -lm -g && ./build/zheng.exe "$@"