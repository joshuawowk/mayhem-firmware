# Recovery

This guide covers how to recover a HackRF/PortaPack device that doesn't come up correctly after flashing, or that has a firmware bug that leaves it unusable. See [FLASHING.md](FLASHING.md) for the normal install/update process.

Work through the sections in order — most problems are solved by the first one or two steps.

## Quick diagnosis

| Symptom | Likely cause | Go to |
|---|---|---|
| Device not detected at all by the computer / no lights | Bad USB cable/port, or interrupted flash | [Hardware checks](#0-hardware-checks), then [DFU recovery](#2-dfu-recovery-unbricking) |
| Mayhem UI is frozen, garbled, or crash-loops on boot | Bad or incompatible firmware build | [Reflash known-good firmware](#1-reflash-a-known-good-firmware) |
| Mayhem boots but a specific app crashes/hangs it | Corrupted/incompatible app or settings file on the SD card | [Hard Reset app](#3-hard-reset-app-clear-settings--bad-apps) or [SD card recovery](#4-sd-card-recovery) |
| `hackrf_spiflash`/flasher reports a write or verify error | Interrupted flash, bad cable, or USB hub | [Hardware checks](#0-hardware-checks), then retry flashing |
| Screen shows a "FLASH ERR" / checksum error message | Firmware image is corrupt on the flash chip | [DFU recovery](#2-dfu-recovery-unbricking) |
| Want to remove Mayhem entirely | N/A | [Restore stock HackRF firmware](#5-restore-stock-hackrf-firmware) |

## 0. Hardware checks

Before anything else, rule out the most common causes of a "bricked" device:

- Use a different USB cable and a different USB port — prefer a port on the computer itself over a hub or dock.
- If your PortaPack has a physical **battery/power switch**, make sure it's on.
- Try disconnecting the PortaPack and flashing the bare HackRF first, then reattach the PortaPack afterward.
- Make sure no other tool (SDR software, virtual machine, etc.) is holding a connection open to the device.

A device that no longer flashes normally is almost never permanently destroyed — the HackRF's boot ROM has a hardware-level DFU mode (below) that exists specifically for this situation.

## 1. Reflash a known-good firmware

If the currently-installed firmware is unstable, the fastest fix is simply to flash it again, or flash the previous stable release you know worked:

1. Download a known-good firmware from the [Releases page](https://github.com/portapack-mayhem/mayhem-firmware/releases) — pick the latest **stable** release rather than a nightly build if you were running a nightly.
2. Follow the normal flashing steps in [FLASHING.md](FLASHING.md).

If the device still boots (even if the UI misbehaves), you can usually still get it into HackRF mode from the home screen's **HackRF** icon to flash it again. If the UI is completely unresponsive, use DFU recovery below.

## 2. DFU recovery (unbricking)

DFU (Device Firmware Upgrade) mode is a bootloader built into the HackRF's microcontroller, entered with a physical button combo. It works regardless of what firmware (or lack of firmware) is currently on the flash chip, so it's the way to recover a device that won't boot or won't enumerate normally.

1. Unplug the device.
2. Press and hold both the **DFU** and **RESET** buttons.
3. Plug the USB cable in (or, if already plugged in, keep holding).
4. Release **RESET** first, then release **DFU**.
5. The device should now enumerate as a DFU device rather than a HackRF.

Then, either:

- **Windows:** run `mayhem_flasher.bat` from the `flashing` folder and choose action `2` (**Flash DFU then Mayhem**) to unbrick and reflash Mayhem in one go, or action `3` (**Flash via DFU**) to just load a working HackRF firmware into RAM without permanently writing it.
- **Linux/macOS:**
  ```
  dfu-util -D hackrf_usb.dfu          # HackRF One / PortaRF
  dfu-util -D hackrf_hpro_usb.dfu     # HackRF Pro
  ```
  This loads a temporary firmware into RAM (it does not persist across power cycles). Once it's loaded, immediately flash the real firmware permanently before unplugging:
  ```
  hackrf_spiflash -R -w firmware_hackrf.bin
  ```

If the device still isn't recognized in DFU mode, double-check the button order in step 4 (release RESET before DFU) and try a different USB cable/port.

## 3. Hard Reset app (clear settings / bad apps)

Mayhem includes a built-in **Hard Reset** app (**Settings → Hard Reset**) for recovering from a bad configuration or a corrupted external app without needing a computer:

- Clears `.ini` files in the `SETTINGS` folder on the SD card.
- Deletes external apps (`.ppma`/`.ppmp` files in `APPS`) that fail a version/checksum validation check ("bad apps").
- Resets persistent memory (radio/UI settings stored in battery-backed RAM) to defaults — this also triggers a touch-screen calibration on next boot.

Use this when Mayhem boots but is misbehaving (e.g. a specific app won't open, settings seem corrupted) rather than when the device won't boot at all — if the UI never comes up, use [DFU recovery](#2-dfu-recovery-unbricking) or [SD card recovery](#4-sd-card-recovery) instead, since you won't be able to reach the app.

## 4. SD card recovery

Many "firmware" problems are actually caused by the microSD card contents (corrupted filesystem, incompatible/leftover files from an older release, or a failing card):

1. Remove the microSD card and check it on a computer. Run a filesystem check, or reformat it as **FAT32** if you suspect corruption (back up any files you want to keep first).
2. Re-copy the `sdcard/` contents from the firmware release you're running onto the card (see [FLASHING.md](FLASHING.md#4-copy-the-sd-card-content)) — this ensures apps and resources match the installed firmware version.
3. If problems persist with that card, try a different, good-quality microSD card. Very large, very cheap, or counterfeit cards are a common source of hard-to-diagnose issues.

The device will boot without a microSD card at all (with reduced functionality), which is useful for confirming whether the card itself is the cause of a problem.

## 5. Restore stock HackRF firmware

If you want to remove Mayhem/PortaPack support entirely and go back to plain HackRF functionality:

- **Windows:** run `mayhem_flasher.bat` and choose action `4` (**Flash factory HackRF firmware**).
- **Linux/macOS:**
  ```
  hackrf_spiflash -R -w hackrf_usb.bin        # HackRF One
  hackrf_spiflash -R -w hackrf_hpro_usb.bin   # HackRF Pro
  ```

## Still stuck?

- Check the [wiki](https://github.com/portapack-mayhem/mayhem-firmware/wiki) and [existing issues](https://github.com/portapack-mayhem/mayhem-firmware/issues) for your specific symptom.
- Ask in the project [Discord](https://discord.gg/tuwVMv3) — include your device type (H1/H2/H4/Pro/PortaRF), the firmware version you were flashing, and the exact error/behavior you're seeing.
- If you believe you've found a genuine firmware bug (not a one-off flashing issue), open an [issue](https://github.com/portapack-mayhem/mayhem-firmware/issues/new/choose).
