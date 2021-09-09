#!/bin/bash

mkdir -p tmp
pushd tmp
cmake .. -DINSTALL_DIR=$PWD/../Build
make
mkdir -p ../bin
make install
popd

