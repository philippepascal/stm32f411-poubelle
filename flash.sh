#!/bin/sh

if [ -z "$1" ]; then
  echo "No device specified. Listing DFU devices:"
  dfu-util -l
  exit 0
fi

dev="${1}"

mkdir -p target/thumbv7em-none-eabihf/flash

arm-none-eabi-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/stm32f411-poubelle \
  target/thumbv7em-none-eabihf/flash/firmware.bin

dfu-util --device "$dev" \
  -a 0 \
  -s 0x08000000:leave \
  -D target/thumbv7em-none-eabihf/flash/firmware.bin