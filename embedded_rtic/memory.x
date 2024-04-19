/* Linker script for the nRF52 - WITHOUT SOFT DEVICE */
MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH  : ORIGIN = 0x00000000, LENGTH = 1024K
  /* 
    Data ram, needed for uninit data 

    Note that this is read via the dcode bus not the icode bus.
  */
  RAM    : ORIGIN = 0x20000000, LENGTH = 64K
  /*
    Allocate memory for code in ram (this is icode ram).
  */
  RAM2 	 : ORIGIN = 0x00810000, LENGTH = 64K
  TEXT 	 : ORIGIN = 0x00820000, LENGTH = 64K

  /* This is where we are going to store the control signal buffer */
  BUFFER : ORIGIN = 0x20030000, LENGTH = 16K
}
