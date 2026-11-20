# cw-carrier

The Black Pill makes the CW carrier itself, so the Pi does not have to stream transmit audio
while receiving or in CW. Started 2026-11-20 for the Pi Zero 2 W, whose USB loses receive
when the radio's receive and transmit streams run together.

Separate from the current work: copies of

    pill-firmware/        Black Pill firmware 3.24 (psoc-replacement-stm32 cb9f94a, firmware/pio-async)
    ms-sdr-linux/         Pi ms-sdr 3.215           (mscc-station a67dd310, rpi/)
    SDRcore-recv-linux/   Pi sdrcore-recv 3.157
    SDRcore-trans-linux/  Pi sdrcore-trans 3.163

Nothing here goes back into those repos until Ron says so.

Plan:
- Pill: in CW, with the option on, play a stored 1 ms carrier block (12 kHz at 96 kHz =
  12 cycles = 96 stereo samples) instead of the USB transmit audio. Shaping, key delay,
  PA timing, IQ swap and the one-sample delay stay as they are.
- trans builds that block as it builds the carrier today (level + transmit IQ correction);
  ms-sdr sends it to the pill with a new vendor command at CW select, band and power changes.
- trans, option on: stops its output stream while receiving and in CW, starts it at PTT for
  voice and digital.
- recv, option on (voice and digital, Monitor off): stops its receive stream at PTT and starts
  it again at the end of transmit. With Monitor on, receive keeps running (own signal heard and
  shown), so both streams run then. Cost: a short restart delay back to receive (to measure).
- Result: the radio's receive and transmit streams never run together, except with Monitor on.

## Status (2026-11-20, release cw-R11-20-0)

Code written and built. NOT flashed and NOT run yet: a test build.

    package         mscc 1.0.102+cw1 (= normal mscc 1.0.102 + the parts below)
    pill-firmware   3.90  release/BlackPill-25MHz-3.90-app.bin (+ -boot-app.bin)
    ms-sdr          3.242
    sdrcore-trans   3.190
    sdrcore-recv    3.190

Servers built on the Pi Zero 2 W (binaries/), package made by deb/build-cw-deb.sh.
Checklist for a release: CLAUDE.md ("cw-finished").

Switch: `CW_CARRIER=1;` in `~/.local/mscc/mscc.ini`. Without it (or with an older pill)
everything runs as before.

What each part does:

- pill (`src/audio.c`, `src/usb_vendor.c`): command 0x15. OUT wValue 0..5 + 64 bytes = one
  piece of the 384-byte block, piece 5 makes it active; OUT wValue 0xFF, no data = clear;
  IN 1 byte = status (bit 0 present, bit 1 block active). In CW with a block it plays the
  block (USB transmit audio read and thrown away). USB disconnect clears the block.
- ms-sdr (`source/cw_carrier.c`): at startup asks the pill (0x15 IN); if supported, tells
  trans and recv (0x15 value 1). Passes trans's block to the pill, checks it, answers trans
  0x15 value 2. Every 5 s in CW checks the pill still has a block. Stop_all clears it.
- trans (`sources/cw_carrier.c`): captures 96 frames of its real CW output after 200 ms of
  steady CW and sends them. Any other command from ms-sdr in CW = new capture. I/Q stream
  runs only for voice / digital / tune transmit (and the flush after it) and in CW until
  the pill has a fresh block.
- recv (`sources/cw_carrier_rx.c`): receive stream stopped during voice / digital / tune
  transmit with Monitor off; started again 300 ms after transmit ends.

Test plan (Zero, servers from /tmp/cwc-bin; then proficio):

1. Pill 3.90 with CW_CARRIER off: everything as 3.24 (receive, SSB, FT8, CW, tune).
2. CW_CARRIER=1, CW: log shows "Cw_carrier_startup. ON", "block in the radio", trans
   "I/Q stream stopped". Key the rig into a dummy load: power and spectrum as before
   (compare with the SWR meter), no clicks; change power and band, key again.
3. Receive in CW and SSB with the stream stopped: audio-stats on the Zero, no bursts.
4. FT8 / SSB transmit: start of each transmission not cut, no splatter; receive back
   after transmit; Zero stats during and after transmit.
5. Stop the servers with the rig on: the key must not send RF.
