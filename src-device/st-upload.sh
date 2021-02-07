#!/bin/bash

set -e

st-flash write build/audiomux.bin 0x8000000

