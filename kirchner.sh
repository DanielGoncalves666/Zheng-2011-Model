#!/bin/bash

gcc -o build/kirchner.exe src/*.c -lm -g && ./build/kirchner.exe "$@"