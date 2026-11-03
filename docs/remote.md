# Remote controller: controller on the PC, Pi as I/O server (brief Stage 2b)

Asked for by Simon on 2026-10-19: "a program to execute on the computer where it communicates with the pi for inputs and
back for outputs. Assume worst case latency (try and test what that is) to keep delay constant."

## How it works

```
 Pi (pendctl, 2 ms SCHED_FIFO loop)                                   PC (py/remote_ctrl.py)
 tick k: drain replies -> supervisor -> command = PC's reply for       on every SAMPLE: PID on (theta, omega, v)
         sample k-N (constant delay N = remote.delay_ticks)            -> REPLY(seq k, a) at once
         -> Axis -> SAMPLE k (seq, theta, omega, x, v, ENGAGED?) --->
```

- **Only the PID moves to the PC.** Calibration, homing, the supervisor (engage, theta_abort, soft limits, switches,
  encoder faults, overruns), braking, the operator keys and all logging stay on the Pi, as in `./demo.sh`.
- **Constant delay.** At tick k the Pi applies the command the PC computed from the sample of tick k − N. A reply
  that arrives after 0.3 ms or after 1.9 ms is applied at the same tick. So the delay is N ticks every time, not
  whatever the network did. `remote_constant_delay_equals_the_local_delay_line_under_jitter` checks this
  bit-for-bit: with reply latencies jittering between 1 and N ticks, the commands equal the local PID through the
  Stage 2a delay line of N samples.
- **When a reply is late or lost,** that tick holds the previous command and is counted (`r_miss`).
  `remote.max_miss` (5) ticks in a row abort the run, and so does no reply for `remote.link_timeout` (0.1 s)
  while ARMED or ENGAGED (`Reason::RemoteFault`, "remote link"). This is the normal controlled stop on the Pi.
  Arming is refused while the link is down.
- **Same gains on both sides.** The Pi sends its PID parameters (PARAMS) on HELLO and every 0.5 s. A reply counts
  only if it carries the id of those parameters, so the PC cannot run stale gains. The PC resets its PID at each
  new engagement (epoch), with the integrator at the sample's `i0`: the Pi's pre-load for that engagement (the
  swing-up hand-over's θ̇/ω₀², else 0). Protocol version 2 (2026-11-03) added `i0`; a version-1 PC program is
  ignored, so with an old copy of `py/remote_ctrl.py` the link never comes up (pull it on the PC).
- **Shadow check.** While engaged, the Pi also runs its own PID on the same samples (`a_local`) and logs
  |PC − local| per reply. It is 0 when the network loses nothing (Python and C++ PID are golden-matched), so any
  scaling or sign error in the chain would show at once.
- **Design choices:**
  - UDP, so a late packet never holds up the next one (brief §7).
  - The PC reacts to packets rather than pacing itself with `sleep()`.
  - Kernel receive timestamps on the Pi make every round trip exact rather than rounded to ticks.
  - Packets are little-endian, fixed size and packed byte by byte (`src/common/remote_proto.h`).
  - Golden bytes are shared by the C++ and Python sides (`tests/golden/remote_packets.txt`).

## Remote swing-up (`./swingup.sh --remote`, 2026-11-03)

The pumping, the energy removal and every abort stay on the Pi (`rt::SwingUpRuntime`); only the BALANCE phase after
the hand-over runs on the PC, with the same constant delay as hand catches.
- **Before the hand-over** the samples are not flagged controller, so the PC answers 0. The first `delay_ticks`
  BALANCE ticks therefore apply 0, then the PC's command from the sample `delay_ticks` earlier. That is exactly the
  local PID through a delay line of N samples (`remote_swingup_balances_on_the_pc_with_the_constant_delay`: every
  BALANCE tick bit-identical to the shadow PID N ticks earlier, under jitter 1..N).
- **The pre-load** θ̇/ω₀² goes to the PC as the epoch's `i0`. Without it the simulated catches of the 10-02-18
  hand-over states end sooner (`py` study below).
- **Link:** READY (the go for a run) waits for the link. Link down while pumping: energy out ("remote link down"),
  no hand-over into a dead link. In BALANCE, `remote.max_miss` held ticks in a row or the link going down: brake,
  then energy out ("remote link").
- **Does the catch survive the delay?** The six rig hand-overs of `2026-11-03T10-02-18_balance` (θ ±8.9–9.2°,
  θ̇ 52–63°/s, cart −10…+55 mm at 69–192 mm/s) simulated with the 11-02 plant, the catch-study model:

  | hand-over | local | remote 6 ticks | remote 6, no pre-load | remote 10 ticks |
  |---|---|---|---|---|
  | +9.1°, −61°/s, +22 mm, +174 mm/s | soft limit 2.7 s | 5 s | soft limit 1.3 s | 5 s |
  | +9.2°, −56°/s, +55 mm, +191 mm/s | 5 s | soft limit 4.6 s | 2.0 s | 4.6 s |
  | +9.1°, −52°/s, +52 mm, +192 mm/s | 2.6 s | 2.8 s | 3.0 s | 2.9 s |
  | +9.1°, −62°/s, −3 mm, +69 mm/s | 0.87 s | 0.93 s | 0.66 s | 0.91 s |
  | +8.9°, −63°/s, +8 mm, +69 mm/s | 0.89 s | 0.91 s | 0.63 s | 0.82 s |
  | −8.9°, +55°/s, −10 mm, −134 mm/s | 2.5 s | 2.9 s | 1.2 s | 3.5 s |

  No fall in any case (5 s = the end of the simulation). 6 ticks changes the run lengths by less than their spread
  between hand-overs; dropping the pre-load shortens five of the six. (The model's runs are shorter than the rig's
  30 s ones; it is the comparison that counts.)

## Choosing the delay: measure the worst case

The delay N must cover the worst round trip (Pi sample → PC → reply at the Pi) on the PC and network you use:

```
N = smallest number of ticks with  worst round trip < N * 2 ms - 0.1 ms,   then + 1 tick of margin
```

Each tick of delay costs 2.1° of the PID's 46° phase margin (ω_gc ≈ 18.6 rad/s). The PID tolerates ~43 ms in
all (Gate B; 37 ms if the delay varies), so N up to ~10 keeps a comfortable margin.

Measured so far:

| link | how | round trip p50 / p99.9 / max (ms) | smallest N that covered it |
|---|---|---|---|
| localhost, WSL on Simon's laptop | `tests/remote/run_remote_e2e.sh` (C++ link + PC program) | 0.13 / 0.63 / 1.48 | 1 |
| laptop Wi-Fi → router → Pi wired, 10 s | `py/net_probe_*` (Python both ends) | 1.27 / 8.0 / 11.2 | 6 (→ delay_ticks 7) |
| Simon's wired PC → Pi (2026-11-03, 1.4 min, 42 085 replies, 0 lost) | `remote-sim` + `py/remote_ctrl.py` (`logs/2026-11-03T14-14-50_remote-sim`) | 0.46 / 2.39 / 4.17 | 3 (→ 4) |
| same wired PC, first rig session (2026-11-03, 5.8 min, 172 907 replies, 0 lost) | `./remote.sh` (`logs/2026-11-03T14-32-03_balance`) | 0.41 / 2.50 / 13.2 (clustered stalls) | 7; 6 leaves 2 single late ticks → delay_ticks **6**, now the default |

A 10-minute Wi-Fi run ended when the laptop left the lab network. On Wi-Fi the worst case includes the link
dropping out entirely, which no delay covers; the brief says wired Ethernet.

**The Windows PC program** (2026-10-20, laptop, Windows Python → simulated Pi in WSL). This is its answer time,
receive → send, i.e. its own part of the round trip:

| priority | samples | p50 | p99 | p99.9 | max | answers > 1 ms |
|---|---|---|---|---|---|---|
| normal | 30 702 (60 s) | 47 µs | 236 µs | 1.0 ms | **15.9 ms** | 32 |
| 1 ms timer only | 12 755 (25 s) | 56 µs | 193 µs | 408 µs | 5.0 ms | 5 |
| high class + time-critical thread + 1 ms timer (**default now**) | 41 793 (84 s) | 54 µs | 140 µs | 253 µs | **0.99 ms** | 0 |

At normal priority Windows sometimes takes the CPU away for a whole scheduler slice (~16 ms), which would decide
the worst case. The default settings apply to this process only and are undone when it exits; `--normal-priority`
switches them off. The WSL "Pi" in these runs is not real-time: its loop stalled up to 41 ms and its virtual
network held packets for up to 111 ms. Two runs therefore ended with the remote-link abort, exactly as designed.
So the round trips of these runs say nothing about the real Pi; only the PC's answer time above carries over.

## Procedure (wired PC)

1. **Latency, no motion** (simulated pendulum on the Pi, no GPIO; the real loop, link and PC program):
   - PC: `python py/remote_ctrl.py --host pendel.local`
   - Pi: `make build/pendctl && build/pendctl --mode remote-sim --set remote.delay_ticks=10 --set remote.sim_seconds=600`
     (`sudo` for SCHED_FIFO timing, as on the rig).
   - Then on the PC: `py/fetch_runs.py` and `py/remote_report.py logs/<time>_remote-sim`. The report prints the
     round-trip distribution, the late replies for every N, and the delay_ticks to use.
   - A long run is lost if the ssh session drops; use `tmux` or `nohup` for 10 minutes.
2. **Set the delay:** `remote.delay_ticks` in `config/pendctl.conf`, or `--set` on the command line.
3. **A second remote-sim run with that delay:** it must complete with 0 missed ticks.
4. **Rig (HARDWARE ACTION, with Simon's go):** PC `python py/remote_ctrl.py`, Pi `./remote.sh`.
   - It runs the same flow as `./demo.sh`: calibration swing, Enter homes, arms when the link is up and the pendulum
     hangs still, lift, let go at the beep.
   - The status lines show the PC's worst round trip, late replies and missed ticks.
   - Logs: `data.csv` (+ `ctrl, epoch, a_local, r_miss`), `remote.csv` (one row per reply: seq, rx_tick, rtt_ns,
     a_cmd, a_local, pc_proc_ns) and the `remote_*` fields in `meta.json`. The PC writes
     `logs/<time>_remote-pc/samples.csv` and `summary.json`.

Windows: the PC program needs only Python 3 (no packages). It raises its own process priority by default (see
above). If Windows Firewall asks about Python, allow private networks (Simon: fine). Replies to the PC's own HELLO
normally pass without a rule.

## Files

- Pi:
  - `src/common/remote_proto.h`: packets.
  - `src/runtime/remote.h`: the command-source interface.
  - `src/runtime/balance.{h,cpp}`: remote branch, misses, `RemoteFault` input.
  - `src/core/supervisor.*` and `py/pendel/supervisor.py`: `RemoteFault` = 16.
  - `src/app/remote_link.{h,cpp}`: UDP and kernel timestamps.
  - `src/app/balance_mode.cpp`: `balance.remote`.
  - `src/app/remote_sim.cpp`: `--mode remote-sim`.
  - `remote.sh`.
- PC: `py/remote_ctrl.py`, `py/remote_report.py`, `py/net_probe_pi.py` + `py/net_probe_pc.py` (network-only
  probe, Python on both ends).
- Tests:
  - `tests/unit/test_remote.cpp`: golden bytes, the constant-delay equivalence, a late reply held, abort after
    max_miss, link down refuses arming and aborts ARMED, config checks.
  - `tests/remote/run_remote_e2e.sh`: C++ link + PC program over localhost; WSL or the Pi.

## Open

- The wired-PC measurement (procedure step 1) is done (2026-11-03, above). Open: a 10-minute confirming run at
  delay_ticks 4 (`--set remote.sim_tap_period=0 --set plant.coulomb=0`: the simulated 11-02 pivot friction sticks the rod
  at upright and the cart drifts off, and taps make the angle-only drift end a run within ~1.5 min), and the first rig run.
- On the Pi: `make` and `make test`, the gpio-mockup balance test (local mode unchanged?), a short remote-sim.
- Late replies count by tick, so after a loop overrun the catch-up ticks shrink the delay in time. On the Pi
  overruns are aborts (3 in a row) and have not happened in 1.26 M logged ticks; under WSL the loop stalled 49 ms
  once.
- Remote swing-up is built and tested on the PC (91/91) but not yet run on the rig. The pre-load at release for
  hand catches (`ctrl.preload` with `sup.engage_on_release`) is no longer refused in remote mode (it goes as `i0`),
  but it is untested there and stays off by default.
- `remote-sim` simulates the configured pivot friction; with the 11-02 pivot pass `--set plant.coulomb=0` (the rod
  sticks at upright and the angle-only PID drives the cart off).
