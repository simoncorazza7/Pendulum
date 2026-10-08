# Stage 0.1: discovery (2026-10-08)

All read-only. Nothing was built, run, or changed on the Pi except creating the git repo (approved).

## Pi
| Item | Value | Notes |
|---|---|---|
| Board | Raspberry Pi 4 Model B Rev 1.5, **1 GB** RAM (906 MiB) | thesis BOM linked a 4 GB model |
| OS | Raspberry Pi OS, Raspbian 12 (bookworm), **32-bit armhf userland**, 64-bit kernel | image "Raspberry Pi reference 2026-10-06", i.e. freshly imaged |
| Desktop | `graphical.target`, labwc (Wayland) + pcmanfm running | competes for CPU; consider for 0.5 |
| Kernel | `6.12.109+rpt-rpi-v8 #1 SMP PREEMPT` | full preemption, **not** PREEMPT_RT; no `isolcpus`/`nohz_full` |
| RT limits | `sched_rt_runtime_us` 950000 / period 1000000; `ulimit -r` 0; memlock 116048 kB | RT throttling active (95 %); a non-root user can't use SCHED_FIFO |
| CPU | 4 cores, governor `ondemand`, 600–1800 MHz, `arm_boost=1` | frequency scaling adds latency; revisit in 0.5 |
| Health | `throttled=0x0`, 43.8 °C, core 0.926 V (idle, 12 min uptime) | |
| libgpiod | 1.6.3 (C + C++ bindings, tools), v1 API | no port needed |
| pigpio | `pigpiod` installed, inactive and disabled | would conflict if started |
| GPIO chips | `gpiochip0` = pinctrl-bcm2711 (58), `gpiochip1` = raspberrypi-exp-gpio (8) | |
| `gpiochip4` | `/dev/gpiochip4 → gpiochip0` symlink from `/usr/lib/udev/rules.d/60-gpiochip4.rules` | `gpiodetect` opens it fine (lists gpiochip0 twice) |
| Pin claims | BCM 2–27 all `unused input`; no I²C on GPIO2/3 (only `/dev/i2c-20/21` = HDMI DDC); no SPI devices | GPIO2/9/10 are free |
| Permissions | user `pendel` is in `gpio`, `i2c`, `spi`, `sudo` | GPIO works without sudo; RT priority needs sudo or limits |
| Toolchain | g++/gcc 12.2.0, git 2.39.5, Python 3.11.2 | **no cmake, no gnuplot** |
| Network | **eth0** 192.168.0.100/24 (wired); wlan0 down | PC is on Wi-Fi "birdwatcher" (192.168.0.103) |
| Autostart | none (no crontab, no service, no autostart entry); port 8080 closed | |
| Disk | 23 GB free of 29 GB | |

## Code on the Pi (`~/Pendel-main`)
Not a git repo before today. It contains the sources plus a prebuilt `main` (ELF 32-bit ARM, 2026-10-07 13:29).
Diff against upstream `eb760fe` (2026-03-30):

| File | Change on the Pi |
|---|---|
| `axis.h` | adds the missing `#endif` of the include guard (upstream doesn't compile) |
| `encoder.cpp` | `winkelSchritte = winkelSchritteP;` commented out (undeclared member; suspected issue 11) |
| `start.sh`, `Kompilierer.sh` | file modes only (start.sh +x, Kompilierer.sh −x) |

So upstream doesn't build as published, and the Pi copy is upstream plus the minimum to compile.

## Repo layout created
- Pi `~/Pendel-main`: branches `pi-baseline` (= as-found snapshot `345ab78` on top of upstream history) and
  `control-dev` (working branch). `receive.denyCurrentBranch=updateInstead`, so a push from the PC
  updates the Pi's working tree. Remote `upstream` has its push URL disabled.
- PC `C:\Users\simon\Downloads\Pendel-main\Pendel-main`: same repo, remotes `pi` and `upstream`
  (push disabled), `core.autocrlf=false` (files stay LF).

## Bearing on suspected issues (brief §2)
- **10** (`gpiochip4`, libgpiod 2.x): not a problem on this OS. The symlink exists and v1.6.3 is installed.
- **11** (`winkelSchritte`): confirmed. Upstream fails to compile, and the Pi carries a fix.
- **12** (gnuplot/SIGPIPE): gnuplot is **not installed**, so the failure path is live. To be checked in code reading.
- **9** (no RT): confirmed at the OS level (PREEMPT, not RT; RT throttling at 95 %; ondemand governor).

## Thesis
See `thesis_notes.md`.
