#!/bin/bash

gcc -O2 -o build/kirchner.exe src/*.c -lm -g && ./build/kirchner.exe "$@"