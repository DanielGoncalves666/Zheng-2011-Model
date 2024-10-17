#!/bin/bash

gcc -Wall -o build/zheng.exe src/*.c -lm -g && ./build/zheng.exe "$@"