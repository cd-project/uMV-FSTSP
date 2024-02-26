#!/bin/bash
n_customer=10
dtl=20
binary_file="/mnt/c/Users/ORG/CLionProjects/uMV-FSTSP/cmake-build-release/uMV-FSTSP"

for i in {1..20}; do
  "$binary_file" $n_customer $dtl $i
done