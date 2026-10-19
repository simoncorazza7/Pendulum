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
  new engagement (epoch).
- **Shadow check.** While engaged, the Pi also runs its own PID on the same samples (`a_local`) and logs
  |PC − local| per reply. It is 0 when the network loses nothing (Python and C++ PID are golden-matched), so any
  scaling or sign error in the chain would show at once.
- **Design choices:**
  - UDP, so a late packet never holds up the next one (brief §7).
  - The PC reacts to packets rather than pacing itself with `sleep()`.
  - Kernel receive timestamps on the Pi make every round trip exact rather than rounded to ticks.
  - Packets are little-endian, fixed size and packed byte by byte (`src/common/remote_proto.h`).
  - Golden bytes are shared by the C++ and Python sides (`tests/golden/remote_packets.txt`).

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
| Simon's wired PC → Pi | **to do**: procedure below | | |

A 10-minute Wi-Fi run ended when the laptop left the lab network. On Wi-Fi the worst case includes the link
dropping out entirely, which no delay covers; the brief says wired Ethernet.

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

Windows: the PC program needs only Python 3 (no packages). `--high-priority` raises its own process priority. If
Windows Firewall asks about Python, allowing private networks is Simon's decision. Replies to the PC's own HELLO
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

- The wired-PC measurement (procedure step 1) and the first rig run.
- Late replies count by tick, so after a loop overrun the catch-up ticks shrink the delay in time. On the Pi
  overruns are aborts (3 in a row) and have not happened in 1.26 M logged ticks; under WSL the loop stalled 49 ms
  once.
- Not in remote mode: swing-up (local only), the integrator pre-load at release (refused at load).
