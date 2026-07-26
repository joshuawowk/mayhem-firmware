# Flashing Mayhem Firmware

This guide walks through installing Mayhem firmware on a HackRF One with a PortaPack (H1, H2, H4/H4M, or the metal-case variants), the HackRF Pro / "Praline", or a PortaRF device.

If you run into trouble, see [RECOVERY.md](RECOVERY.md) for unbricking and troubleshooting steps.

## 1. What you need

- A HackRF One (with or without a PortaPack attached) or a HackRF Pro / PortaRF.
- A USB cable connected directly to your computer (avoid USB hubs, which can cause flashing failures).
- The Mayhem firmware release, downloaded from the [Releases page](https://github.com/portapack-mayhem/mayhem-firmware/releases). Use the latest **stable** release unless you specifically need a **nightly** build.
  - Download the `.zip` for your platform (Windows) or the standalone firmware/SD card files (Linux/macOS), and extract it.

The extracted release contains three relevant folders:

- `firmware/` — the `.bin`/`.dfu` firmware images for each device variant.
- `flashing/` — the flashing tools and (Windows-only) `mayhem_flasher.bat` helper script.
- `sdcard/` — the files to copy onto the PortaPack's microSD card (apps, resources, etc.).

## 2. Put the device into HackRF (flashing) mode

The HackRF's internal SPI flash can only be programmed while the device is running in plain **HackRF USB mode**, not while Mayhem's PortaPack UI is active.

- **If a PortaPack is attached and Mayhem is already running:** on the home screen, tap the **HackRF** icon, then confirm **"Switch to HackRF mode?"**. The device reboots into HackRF USB mode. (To leave HackRF mode and return to Mayhem afterwards, press the physical **RESET** button.)
- **If the device has no PortaPack attached, or Mayhem doesn't boot:** the HackRF automatically enumerates in HackRF USB mode on power-up, so no extra step is needed. If it doesn't, see [RECOVERY.md](RECOVERY.md) for the DFU recovery procedure.

## 3. Flash the firmware

### Windows

1. Open the `flashing` folder from the extracted release.
2. Run `mayhem_flasher.bat`.
3. Select your device:
   - `1` — HackRF One / PortaPack
   - `2` — PortaRF
   - `3` — HackRF Pro / PortaPack
4. Select the action:
   - `1` — **Flash Mayhem firmware** (normal update, use this in most cases)
   - `2` — **Flash DFU then Mayhem** (does a DFU unbrick first, then flashes Mayhem in one go — use if your device isn't detected normally)
   - `3` — **Flash via DFU** (unbrick only — loads HackRF firmware into RAM without writing Mayhem)
   - `4` — **Flash factory HackRF firmware** (removes Mayhem/PortaPack support entirely, restores stock HackRF firmware)
5. Follow the on-screen prompts. If the script can't detect the device over USB, it will try to fall back to switching it into HackRF mode automatically over a serial port.
6. Wait for `Programming... [######] Done` (or similar success output), then unplug and replug the device.

If you have driver problems on Windows (device not recognized, shows up as "Unknown Device" in Device Manager), install the drivers from `flashing/driver/dpinst.exe` first.

### Linux / macOS

There is currently no equivalent of `mayhem_flasher.bat` for Linux/macOS, so the HackRF command-line tools are used directly.

1. Install the HackRF host tools and `dfu-util`:
   - Debian/Ubuntu: `sudo apt install hackrf dfu-util`
   - macOS (Homebrew): `brew install hackrf dfu-util`
2. Put the device into HackRF mode (see step 2 above), then confirm it's visible:
   ```
   hackrf_info
   ```
3. From the extracted release's `firmware/` folder, flash the image that matches your device:
   ```
   hackrf_spiflash -R -w firmware_hackrf.bin     # HackRF One / PortaPack
   hackrf_spiflash -R -w firmware_portarf.bin    # PortaRF
   hackrf_spiflash -R -w firmware_hpro.bin       # HackRF Pro / PortaPack
   ```
   `-w` writes the image, `-R` resets the device once the write finishes.
4. Wait for the command to finish without errors, then unplug and replug the device.

## 4. Copy the SD card content

Mayhem's apps and resources (frequency databases, TTS voices, SD-card apps, etc.) live on the PortaPack's microSD card, separately from the firmware image itself.

1. Remove the microSD card from the PortaPack and mount it on your computer (format as FAT32 if it's new/blank).
2. Copy the **contents** of the release's `sdcard/` folder to the **root** of the microSD card, merging/overwriting existing folders (`APPS`, `FREQMAN`, `WAV`, etc.) rather than replacing the whole card, so you don't lose your saved files.
3. Safely eject the card and re-insert it into the PortaPack.

## 5. Verify

Power on the device. Mayhem should boot to its home screen, and the version shown in **System info** (or the top of the boot screen) should match the release you flashed.

If the device doesn't boot, shows a checksum/flash error, or is not detected at all, see [RECOVERY.md](RECOVERY.md).

## Notes for HackRF-only users

Flashing Mayhem onto a bare HackRF One (no PortaPack attached) is safe — the extra PortaPack UI code simply stays dormant until a PortaPack is connected. If you later want to go back to stock HackRF-only firmware, use action `4` in `mayhem_flasher.bat`, or run:

```
hackrf_spiflash -R -w hackrf_usb.bin        # HackRF One
hackrf_spiflash -R -w hackrf_hpro_usb.bin   # HackRF Pro
```
