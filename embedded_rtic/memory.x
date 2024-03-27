/* Linker script for the nRF52 - WITHOUT SOFT DEVICE */
MEMORY
{
  /*
	This memory map is based on https://infocenter.nordicsemi.com/index.jsp?topic=%2Fps_nrf52840%2Fmemory.html

  */
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x00000000, LENGTH = 1024K
  RAM 	: ORIGIN = 0x00800000, LENGTH = 64K
  RAM2 	: ORIGIN = 0x00810000, LENGTH = 64K
  TEXT 	: ORIGIN = 0x00820000, LENGTH = 64K

  /* This is where we are going to store the control signal buffer */
  BUFFER : ORIGIN = 0x00838000, LENGTH = 32K
}
