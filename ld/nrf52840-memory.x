/* nRF52840 memory layout — Adafruit UF2 bootloader + s140 SoftDevice
 *
 *   0x00000 – 0x01000   MBR (Master Boot Record)
 *   0x01000 – 0x26000   s140 SoftDevice (dormant — we don't use BLE)
 *   0x26000 – 0xEA000   Application  ← we live here
 *   0xEA000 – 0xF4000   Reserved (app data)
 *   0xF4000 – 0xFC000   UF2 Bootloader
 *   0xFF000 – 0x100000  Bootloader settings
 *
 * RAM: first 8 bytes reserved for MBR interrupt forwarding. SoftDevice
 * RAM (typically 0x20000008–0x20002000) is NOT reserved here because we
 * never call sd_softdevice_enable — SD sits dormant in flash and never
 * touches RAM. cortex-m-rt's set-vtor feature handles relocating VTOR
 * to the app's vector table at 0x26000 at runtime.
 */

MEMORY
{
  FLASH : ORIGIN = 0x00026000, LENGTH = 0xCE000
  RAM   : ORIGIN = 0x20000008, LENGTH = 255K
}
