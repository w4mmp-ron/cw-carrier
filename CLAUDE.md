# CLAUDE.md: cw-carrier

Claude's rules for this repo (Ron, W4MMP). Separate from mscc-station and the pill repo
(psoc-replacement-stm32): nothing here goes into them, and nothing of theirs is edited from
here, until Ron says so. The plan, what each part does and the test plan are in README.md.

## Keywords

| Ron says | Claude does |
|---|---|
| "cw-build" | Builds only (pill, servers, .deb). Nothing committed, pushed or published. |
| **"cw-finished"** | Runs the checklist below: builds, .deb, docs, one commit, push, GitHub release. |
| "stop" | Stops at once. |

The normal "finished" / "build" keywords belong to mscc-station; they never touch this repo,
and "cw-finished" never touches mscc-station or the pill repo.

## Working style (same as mscc-station)

- Pull first: `git -C C:/Users/Ron/.grok/cw-carrier pull --rebase --autostash`.
- Ask before editing: diagnosis + proposed fix, edit once Ron says OK.
- Nothing goes into git before "cw-finished". One commit per job, specific paths.
- Never flash the pill unless Ron says so.

## Trees

| Path | What |
|---|---|
| `pill-firmware/` | Black Pill firmware (copy of pio-async 3.24). Build output: `pill-firmware/release/`. |
| `ms-sdr-linux/`, `SDRcore-recv-linux/`, `SDRcore-trans-linux/` | Pi servers (copies of mscc-station `rpi/`). |
| `binaries/` | The server binaries that go into the .deb (built on the Zero). |
| `deb/build-cw-deb.sh` | Makes the .deb from a normal mscc release .deb. |
| `installers/` | Built .debs (not in git). |

## Versions

- Pill: `FIRMWARE_VERSION_MINOR` in `pill-firmware/include/proficio_config.h`, 90 and up
  (3.90 = 3.24 + stored carrier). Bump it for every build that gets flashed or shipped (the
  build saves a changed rebuild of the same version as `-b2`).
- trans / recv: `VERSION_MINOR` in `sources/extern.h`, 190 and up, bumped by hand.
- ms-sdr: bumps its own `VERSION_MINOR` on every build (240 and up); the build's copy goes
  back into `ms-sdr-linux/source/version.h` (build -> repo only).
- Package: `<normal mscc version>+cw<n>`, e.g. 1.0.102+cw1.
- Keep these numbers clear of the normal ones (pill 3.2x, trans 3.16x, recv 3.15x, ms-sdr 3.21x).

## "cw-finished" checklist

1. **Pull** this repo. Check the normal mscc release: if there is a newer `mscc_<ver>_arm64.deb`
   in `C:/Users/Ron/.grok/worktrees/installers/rpi/`, use it as the base (and say so).
2. **Pill** (if changed): bump the minor, then
   `cd pill-firmware && C:/Users/Ron/.platformio/penv/Scripts/pio.exe run`
   -> `pill-firmware/release/BlackPill-25MHz-3.<minor>-app.bin` + `-boot-app.bin`. No warnings.
3. **Servers** (if changed): bump trans / recv by hand. Build on the Zero, straight into
   `~/mscc` (Ron 2026-11-20: the Zero is the test box, so the build replaces its running
   servers): stop the servers first (`mscc stop`; a running binary can't be overwritten),
   copy the three trees to `zero2:~/cw-carrier` (tar over ssh, CR stripped from text files),
   then `cd ~/cw-carrier/<tree> && make -j3`. Never on proficio (its `~/mscc` is the normal
   mscc). `make clean` deletes the installed server in `~/mscc`: only right before a make.
   Copy `~/mscc/{ms-sdr,sdrcore-recv,sdrcore-trans}` into `binaries/` and the Zero's
   ms-sdr version number back into `ms-sdr-linux/source/version.h`.
4. **.deb**: bump `+cw<n>`, then
   `wsl bash /mnt/c/Users/Ron/.grok/cw-carrier/deb/build-cw-deb.sh <base.deb> <version>`.
   It lists every file that differs from the base: only control, the changed servers and the
   new pill files may differ. File count = base + the new pill files.
5. **Docs**: README.md status section (versions, what changed, what is tested), this file
   if a rule changed.
6. **One commit, then push** (this repo only). The message says what changed and what is
   tested / not tested.
7. **GitHub Release** `cw-R<month>-<day>-<n>` in w4mmp-ron/cw-carrier:
   `gh release create cw-R.. -R w4mmp-ron/cw-carrier --title cw-R.. --target main --notes-file notes.md installers/mscc_<ver>_arm64.deb pill-firmware/release/BlackPill-25MHz-3.<minor>-app.bin pill-firmware/release/BlackPill-25MHz-3.<minor>-boot-app.bin`.
   Notes: short version table (pill, ms-sdr, trans, recv, package, base mscc), what changed,
   "Downloads: which file do I need?" (the .deb = Raspberry Pi; -app.bin = flash with the Pi's
   Black Pill Firmware Update window; -boot-app.bin = ST-Link, new pill only), how to turn it
   on (`CW_CARRIER=1;` in `~/.local/mscc/mscc.ini`), and that it is a test build. Check the
   uploaded sizes. Keep the newest release and one earlier one; delete older ones (keep the
   git tags).

If nothing installable changed, "cw-finished" is just the commit and push.

## Installing on a Pi (same rule as the normal mscc)

    scp installers/mscc_<ver>_arm64.deb <pi>:/tmp/
    ssh <pi> 'sudo -n env DEBIAN_FRONTEND=noninteractive apt install -y /tmp/mscc_<ver>_arm64.deb'

Never plain `sudo apt install`: the postinst asks about stopping the servers and waits.
Back to the normal package: install the normal mscc .deb the same way with
`--allow-downgrades`.
