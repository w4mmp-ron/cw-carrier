# PC13 LED Diagnostic Codes — pio-async

Quick reference for the on-board LED (PC13) on the WeAct Black Pill running
the `pio-async` firmware (`release/BlackPill-25MHz-<version>-app.bin`, or `BlackPill-25MHz-<version>-boot-app.bin` with the bootloader; before 2026-11-11 `app-firmware-async.bin` / `boot-app-firmware-async.bin`).

## What you see after power-up

1. **Dark for about 2 s**: the codec and I2S start at power-up (clocks, codec
   reset, register setup, settling), then the LED stays dark for one pause.
2. **Solid ON = normal** (3.15+): USB enumerated by the host and the audio is
   fine (would be **O**, or **T** while transmitting).
3. Otherwise **a Morse letter, repeating**: the current condition (table below).

The codec/I2S audio system runs from power-up **whether or not USB is
connected**, so the letters show even with the Pi / PC off. With the audio fine
but USB not enumerated (host off, cable out) the LED sends **O** instead of
going solid.

The letter repeats with a pause between repeats. If the condition changes, the
new letter appears on the **next repeat**; a letter is never cut off part-way.

## Letter codes (highest priority first)

| Letter | Morse | Meaning | What to check |
|---|---|---|---|
| **C** | `− · − ·` | PCM3060 codec did not answer on I2C | I2C1 wiring (PB8 SCL / PB9 SDA), pull-ups, codec power, codec reset (PA2). **Normal on a bare Black Pill with no transceiver.** |
| **U** | `· · −` | Codec wrote OK but did not answer a register **read** | Codec may not support reads, or I2C read timing. Not fatal by itself. |
| **R** | `· − ·` | Register read-back **differs** from what was written (0x40/0x41/0x42/0x45) | Codec reset / RST (PA2) or power-save not released. |
| **L** | `· − · ·` | SI5351 LO chip did not acknowledge its last I2C write (no LO, so no signals) | SI5351 on I2C1 (address 0x60), its power, I2C wiring. Bench build ignores it. |
| **F** | `· · − ·` | I2S full-duplex DMA failed to start | Firmware problem (I2S/DMA handle not ready). |
| **P** | `· − − ·` | I2S clock PLL (PLLI2S) not locked | Clock configuration (HSE 25 MHz, PLLI2S M/N/R). |
| **N** | `− ·` | I2S peripheral (SPI2 I2SE) will not enable | Firmware / I2S init. |
| **D** | `− · ·` | No I2S bit clock (polled TXE never came back) | I2S clock source / prescaler. |
| **S** | `· · ·` | Bit clock running, but no DMA half-buffer has completed | DMA setup / DMA interrupts. |
| **Z** | `− − · ·` | Audio streaming, but everything received from the codec is zero | Codec not sending audio: PB14 (codec DOUT to MCU), codec clocks (MCLK PA3, BCK PB13, LRCK PB12), codec power/format. **Normal on the bench build.** |
| **K** | `− · −` | Codec data is **frozen**: non-zero, but every sample is the same value (ADC not converting) | ADC clocks at the codec (SCKI1/BCK1/LRCK1), codec setup. Seen 2026-09-24 as a constant -335 that fooled the old O check. |
| **G** | `− − ·` | Codec data changes, but not steadily (glitch or noise on PB14?) | Real ADC data would be steady. Check DOUT (PB14) wiring/grounding. |
| **O** | `− − −` | Codec data **steadily** live: at least 1/8 of the samples in every buffer changing for about 1 s (re-checked each repeat, not latched), **USB not enumerated** | Audio OK; the host has not enumerated the pill (host off, cable, USB). |
| solid ON | (steady) | **Normal** (3.15+): O or T with USB enumerated. Also shown (not expected) if none of the above matches | Normal. |

## While transmitting (keyed by the host, not CW)

While the radio is keyed, the receive letters (Z/K/G/O) are replaced by the state of the
transmit audio coming from the host over USB. Hold the key about 10 s to read the letter.

| Letter | Morse | Meaning | What to check |
|---|---|---|---|
| **M** | `− −` | Keyed, but no audio packets arriving from USB | Firmware USB audio-out path, or the host has not opened the playback stream. |
| **I** | `· ·` | Keyed, packets arriving but every sample is zero | The host is sending silence (power setting, mode, server). |
| **T** | `−` | Keyed, live audio from the host going out to the codec. With USB enumerated (always, in practice) the LED stays **solid ON** instead | **Normal.** If there is still no RF: PB15 → codec DIN, codec DAC, or the radio's TX chain. |

## Bootloader (DFU mode)

**DFU = Device Firmware Update**, the standard USB method for loading new firmware into a
device. In DFU mode the Black Pill is **not running the radio firmware**. It runs only a small
loader program (the bootloader, at the start of flash) that waits for a PC or the Pi to send
new firmware over USB. On the host it shows up as USB device `0483:df11` instead of the
radio. The radio does nothing in this mode; that's normal.

| Letter | Morse | Meaning | What to check |
|---|---|---|---|
| **B** | `− · · ·` | Board is in the **flash DFU bootloader** (not running the app): entered with KEY + NRST, the PA8→GND jumper, or USB command 0xFE | Expected while updating firmware. Burn the app (DFU flasher), then reset without KEY / remove the PA8 jumper. |

B comes from the bootloader (`bootload-PA8`), not the app. Don't confuse it with **D** `− · ·`
(one dot fewer, an app I2S fault).

Counting tip: C, F, P, L and Z have 4 elements; D, S, O, U, R, K and G have 3; N has 2.
O is the only letter that is three long flashes.

## Timing (5 WPM, PARIS standard)

| Element | Duration |
|---|---|
| Dot | 240 ms |
| Dash | 720 ms |
| Gap between dots/dashes | 240 ms |
| Pause between repeats | 1680 ms |

## Normal build vs. bench build

| Build | How | Use |
|---|---|---|
| **Normal** | `pio run` / VS Code env **Default** or `env:blackpill_f411ce` | In the transceiver. Expect **solid ON** (host running), **O** (no host). On a bare Black Pill expect **C**. |
| **Bench** | `pio run -e bench` / VS Code env `env:bench` | Bare Black Pill only. Ignores the codec I2C failure so the I2S/DMA letters show. Expect **Z** (or **O** if the floating data pin picks up noise). **S** means the DMA is not running. |

Bench release copy: `release/app-firmware-bench.bin`.
**Do not use the bench build in the transceiver**: it hides a real codec
failure ("C").

## Results so far

| Date | Build | Hardware | Result |
|---|---|---|---|
| 2026-09-24 | Normal | Bare Black Pill, USB to PC | **C** (expected) |
| 2026-09-24 | Bench | Bare Black Pill, USB to PC | **Z**: full-duplex I2S DMA running |
| 2026-09-24 | Normal | In transceiver, Pi running (codec I2C address 0x23) | **C**: wrong codec I2C address |
| 2026-09-24 | Normal | In transceiver, Pi running (address fixed to 0x46) | **Z**: codec answers, ADC data all zero |
| 2026-09-24 | Normal + register read-back | In transceiver, USB to PC (not Pi) | **O** (old one-sample check) |
| 2026-09-24 | Normal + read-back + steady-RX check | In transceiver, USB to PC (not Pi) | **O**: RX steadily live; read-back matched |
| 2026-09-24 | Normal + read-back + steady-RX check | In transceiver, USB to Pi | **O** |
| 2026-09-24 | Same, arecord capture on Pi | In transceiver | Every sample = **-335** (both channels, 3 s): ADC frozen. The O above was false. |
| 2026-09-24 | + stricter O (K letter) + CSEL1 (ADC on clock set 2) | In transceiver, Pi | **K**: frozen. arecord still -335 both channels |
| 2026-09-24 | + PB14 pull-up | In transceiver, Pi | **K**. arecord still -335: PB14 is actively driven (pull-up had no effect) |
| 2026-09-28 | MCLK 50 % test, `boot-app-firmware.bin` via ST-Link (`flash-bootloader.ps1 -Image combined`) | Bare Black Pill | **C** (expected): combined image burns, bootloader starts the app. `app-firmware.bin` via Win-downloader (DFU) also OK. Transceiver O/K test still to do |
| 2026-10-01 | Keyer address fix build (2026-10-01 00:04), `boot-app-firmware.bin` via ST-Link | Bare Black Pill | App **C** (expected). KEY+NRST: bootloader **B** (Morse-B bootloader works). DFU app burn from B not re-run with this bootloader |
| 2026-10-02 | Same (2026-10-01 00:04) | Black Pill on the daughter board | BOOT jumper (PA8→GND) + NRST: bootloader **B**. The daughter-board BOOT jumper works. DFU app burn from B still not re-run (pill was on the ST-Link, no USB cable to the PC) |
| 2026-10-23 | 3.9 async, `boot-app-firmware-async.bin` via ST-Link | Rev 2 board (partial, no keyer), out of the rig | **C** (expected) |
| 2026-10-23 | Same | Rev 2 board in the rig | **Z**: R11/R12 (MCLK series) were 22 k (mis-marked parts); replaced with 22 ohm -> **O** |
| 2026-10-25 | 3.15 async, `app-firmware-async.bin` via `stm32-app-flash` (Pi, DFU) | In the rig, Pi | **Solid ON** with USB connected; cable pulled -> **O**; plugged back -> solid ON |
| 2026-10-25 | 3.15 | In the rig, Pi | TUN press / release left the LED **off** (host mode change clears CONTROL_LED; solid was written only once) |
| 2026-10-25 | 3.16 (solid LED re-asserted) | In the rig, Pi | **Solid ON** during and after TUN |

Source: `src/app.c` (`compute_diag_led_state()` / `diag_led_poll()`).
