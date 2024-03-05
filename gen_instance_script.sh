#!/bin/bash
n_customer=12
dtl=20
binary_file="/home/cuong/CLionProjects/uMV-FSTSP/cmake-build-release/uMV-FSTSP"

for i in {1..100}; do
  "$binary_file" $n_customer $dtl $i
done