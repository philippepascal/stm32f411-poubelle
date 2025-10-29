/* Flash and RAM layout for STM32F411CEUx */
/* used to be */
/* FLASH : ORIGIN = 0x08000000, LENGTH = 512K */
/* but we are reserving the last sector to store calibration data */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 384K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);