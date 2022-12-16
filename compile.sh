#!/bin/bash

mkdir -p tmp
pushd tmp
cmake .. -DINSTALL_DIR=$PWD/../bin
make
make install
popd

