#!/bin/bash

set -eoux pipefail

lcov --directory /buildroot/build --capture --output-file coverage.info --rc geninfo_auto_base=1 --ignore-errors mismatch,mismatch

lcov --remove coverage.info '/usr/*' '*.test.cpp' '*gtest*' --output-file /buildroot/coverage.filtered.info
genhtml coverage.filtered.info --output-directory /buildroot/coverage-report