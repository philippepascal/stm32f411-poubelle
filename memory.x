/* Flash and RAM layout for STM32F411CEUx */
/* used to be */
/* FLASH : ORIGIN = 0x08000000, LENGTH = 512K */
/* FLASH : ORIGIN = 0x08000000, LENGTH = 384K */
/* but we are reserving the last sector to store calibration data */
/* MEMORY */
/* { */
/*   FLASH : ORIGIN = 0x08000000, LENGTH = 384K */
/*   RAM   : ORIGIN = 0x20000000, LENGTH = 128K */
/* } */

/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */

MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 384K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
  NVM   : ORIGIN = 0x08060000, LENGTH = 128K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* Expose NVM region bounds to Rust */
PROVIDE(_nvm_start = ORIGIN(NVM));
PROVIDE(_nvm_end   = ORIGIN(NVM) + LENGTH(NVM));