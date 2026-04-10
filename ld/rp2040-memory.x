/* RP2040 memory layout — UF2 bootloader (ROM boot2)
 *
 *   0x10000000 – 0x10000100   boot2 (second-stage bootloader, 256 bytes)
 *   0x10000100 – 0x10200000   Application  <- we live here
 *
 * Flash: 2MB (W25Q16JV on Waveshare RP2040-LoRa)
 * RAM: 264KB SRAM (two 256KB banks + 2x 4KB scratch)
 */

MEMORY
{
  BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
  FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
  RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
