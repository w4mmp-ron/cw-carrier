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
