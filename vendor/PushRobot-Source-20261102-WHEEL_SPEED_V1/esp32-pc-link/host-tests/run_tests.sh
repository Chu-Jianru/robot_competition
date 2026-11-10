#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build
g++ -std=c++17 -Wall -Wextra -Werror -I../lib/link_core \
    ../lib/link_core/link_core.cpp test_link_core.cpp -o build/test_link_core.exe
./build/test_link_core.exe
