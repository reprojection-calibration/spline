#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
  bc

# Adopted from https://github.com/svnscha/cpp-coverage-example
lcov --directory /buildroot/build --capture --output-file coverage.info --rc geninfo_auto_base=1 --ignore-errors mismatch,mismatch
lcov --remove coverage.info '/usr/*' '*.test.cpp' '*gtest*' --output-file /buildroot/coverage.filtered.info
genhtml coverage.filtered.info --output-directory /buildroot/coverage-report

coverage_percent=$(lcov --summary /buildroot/coverage.filtered.info | grep "lines" | awk '{ print $2 }' | sed 's/%//')
if (( $(echo "${coverage_percent} < 100" | bc -l) )); then
  echo "Coverage only ${coverage_percent}, please inspect report to get 100% coverage."
  exit 1
else
  echo "Full coverage achieved."
fi