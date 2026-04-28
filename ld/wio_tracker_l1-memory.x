/* Wio Tracker L1 memory layout — UF2 bootloader, NO SoftDevice
 *
 *   0x00000 – 0x01000   MBR (Master Boot Record)
 *   0x01000 – 0xEA000   Application  ← we live here (no SD present)
 *   0xEA000 – 0xF4000   Reserved (app data)
 *   0xF4000 – 0xFC000   UF2 Bootloader
 *   0xFF000 – 0x100000  Bootloader settings
 *
 * Background: Seeed factory firmware ships s140 SoftDevice at
 * 0x01000–0x26000, but our earlier UF2 flashes (with the original
 * `ORIGIN = 0x1000` layout) silently overwrote that region. The WIO
 * bootloader now reports `SoftDevice: not found`; until we restore
 * s140 via SWD, the app must live at 0x1000 to match the bootloader's
 * no-SD boot chain.
 *
 * Delete this file once SD is reinstalled — `build.rs` will then fall
 * back to `ld/nrf52840-memory.x` (the SD-aware default at 0x26000)
 * and the WIO matches every other nRF52840 board.
 */

MEMORY
{
  FLASH : ORIGIN = 0x00001000, LENGTH = 0xE9000
  RAM   : ORIGIN = 0x20000008, LENGTH = 255K
}
