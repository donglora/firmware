/* nRF52840 memory layout — Adafruit UF2 bootloader
 *
 *   0x00000 – 0x01000   MBR (Master Boot Record)
 *   0x01000 – 0xEA000   Application  ← we live here
 *   0xEA000 – 0xF4000   Reserved (app data)
 *   0xF4000 – 0xFC000   UF2 Bootloader
 *   0xFF000 – 0x100000  Bootloader settings
 *
 * RAM: first 8 bytes reserved for MBR interrupt forwarding.
 */

MEMORY
{
  FLASH : ORIGIN = 0x00001000, LENGTH = 0xE9000
  RAM   : ORIGIN = 0x20000008, LENGTH = 255K
}
