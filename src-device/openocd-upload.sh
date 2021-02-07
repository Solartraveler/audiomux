#!/bin/bash

set -e

openocd -f /usr/share/openocd/scripts/interface/stlink-v2-1.cfg -f /usr/share/openocd/scripts/target/stm32f0x.cfg -c "program build/audiomux.bin verify reset exit 0x08000000"

