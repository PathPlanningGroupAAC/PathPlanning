cmake_minimum_required(VERSION 3.8)
project(DelaunayFromScratch)

include_directories(include)

add_library(DelaunayFromScratch DelaunayFromScratch.cpp include/DelaunayFromScratch.h)