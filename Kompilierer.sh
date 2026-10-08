#!/bin/bash
g++ -Wall -std=c++17 -o main main.cpp encoder.cpp axis.cpp pendel.cpp safety.cpp -lgpiod -lgpiodcxx -pthread
