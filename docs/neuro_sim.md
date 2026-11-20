# Evolved recurrent neurocontrollers in simulation (Pasemann 1998, ENS³)

Status 2026-11-19: simulation only, started on the maintainer's request. It is a separate method with its own
script: `py/neuro/`, `py/neuro_check.py`, `py/neuro_evolve.py`, `py/test_neuro.py`, `./neuro_sim.sh`. It changes nothing
in `py/pendel`, the Pi runtime, `demo.sh`, `remote.sh` or `swingup.sh`. Nothing here talks to the Pi.

## How to run (PC)

```
./neuro_sim.sh test                          # 16 checks, incl. rig model = pendel.plant / pid_step / BdLp2 exactly
./neuro_sim.sh check --png docs/img/neuro_benchmark.png
./neuro_sim.sh evolve bench --inputs 4 --gens 1000           # a new run writes logs/<time>_neuro-.../best.json
./neuro_sim.sh evolve rig --rig-inputs 4 --gens 600 --seed 1
./neuro_sim.sh rig py/neuro/nets/rig4_budget_seeded.json --baseline
.venv/Scripts/python py/neuro_check.py replay py/neuro/nets/rig4_budget_seeded.json logs/<time>_balance
```

## 1. The paper reproduces, but only with one reading of its simulator

The weights were decoded from the PDF's text layer with its character spacing honoured. The plain text runs numbers
together, so, for example, w¹'s "15.1 19.97" can otherwise be read as "15.11 9.97". Each network was then run on
the paper's 40 × 40 grid of start states for 120 s.
Simulated with the network's new output applied in the same step and **semi-implicit Euler**, all nine networks
balance 94–97 % of the 1444 squares inside the limits, and they behave as the paper describes. w¹ centres in 4 s.
w⁴'s cart keeps oscillating with a large, constant amplitude. w⁵ settles after 43 s (the paper: under 60 s). w⁸
keeps the pole oscillating.

| net | semi-implicit, output at once | explicit Euler | output one step late |
|---|---|---|---|
| w¹–w³ (4 inputs) | 95–97 % | 95–97 % | 95–97 % |
| w⁴, w⁵, w⁶ (x, θ only) | 96–97 % | 0–1 % | 0 % |
| w⁷, w⁸, w⁹ (x, θ only, s-class) | 94–96 % | 8–49 % | 7–25 % |

![grids](img/neuro_benchmark.png)

The 2-input networks only work because the integrator adds no energy. Their recurrent "velocity estimate" gives
very little damping: w⁴'s self-loop adds about 5 ms of lead, and its pole oscillation takes about 15 s to die out,
as in the paper's figure 6b. Explicit Euler, the formula the explainer doc gives in its section 2, or a single step
of delay is enough to make them fall. Expect the same margin problem on the rig, which has 8 ms of delay.
ENS³ exploits the same thing on the benchmark: within 40 generations it found a 1-neuron, 3-synapse network
(angle plus an inhibitory self-loop) that balances 97 % of the grid.

## 2. The rig model

`py/neuro/rig.py` mirrors `pendel.plant` (pendulum, Axis cap, quantised encoder), `pid_step`'s saturation with the
v_max taper, the Pi's BD+LP2 velocity estimator at 14 Hz, the supervisor's soft limit with braking distance, the
22° abort and a constant command delay. The test checks each of these against the originals exactly.
Validation, with the rig's own controller (`rig.PidPolicy` = `cascade_step` with the config gains), over 200 runs of 20 s:

| controller | training draws | test draws (ref ±2 counts, w0 ±3 %, friction ranges, delay 4–7) |
|---|---|---|
| PID + cart loop (Stage 3b) | 91 % survive, end ~9 mm | 78 % survive, end ~11 mm |
| angle-only PID (Stage 1) | 1 % (drifts into the soft limit by ~2 s) | 0 % |

This matches `py/cascade_study.py` and the rig's history (Stage 1 drifted; the cart loop fixed it).

## 3. Evolution results

The networks start empty, as in the paper. The winning network of every run of 2026-11-19 is kept in `py/neuro/nets/`;
its README lists them. The full run logs stayed in the git-ignored `logs/`.

**Benchmark: ENS³ finds the paper's answers again.** With 4 inputs, after 1000 generations the best network is one
output neuron, w = (0.13, 8.55, 11.90, 15.01, 2.22). That has w¹'s structure and signs and close to its gains (w¹: 10.11,
15.1, 19.97, 2.99), and it balances 1388 grid squares (w¹: 1390). With 2 inputs, after 3000 generations the best network
is w⁴'s architecture, (θ: 23.52, x: 0.34, self-loop −0.58), which balances 1398 squares (w⁴: 1396).

**Rig model.** Each network is scored on 300 fresh runs of 30 s drawn from `rig.TEST`: unseen plants (w0 ±3 %,
b/J 0.03–0.08, Coulomb 0.10–0.25), reference errors to ±2 counts and delays of 4–7 ticks. The runs come from seed 7,
not from the set the winners were picked on.

| controller | survived 30 s | median end abs x | mean abs a | command changes |
|---|---|---|---|---|
| rig PID + cart loop (Stage 3b, `cascade_step`) | 81 % | 6 mm | 0.18 m/s² | 4.9 /s |
| evolved, 4 inputs (x, θ, v, Pi's ω), seeds 1–3 | 85 % each | 14–17 mm | 0.29–0.35 m/s² | 8.0–10.4 /s |
| evolved, 3 inputs (x, θ, v; ω by recurrence), best seed | 52 % | 20 mm | 0.80 m/s² | 7.3 /s |
| evolved, 3 inputs, the other two seeds | 4 %, 1 % | – | – | – |

With the Pi's velocity estimate available, all three seeds converged on the same single neuron with no recurrence,
w = (bias ≈ 0, x 1.0–1.3, θ ≈ 9, v ≈ 2, ω ≈ 1.2). In physical units that is full-state feedback,
a ≈ 88 θ + 12 ω + 17.5 x + 12.5 v. Compare the PID's 63 θ + 6.5 ω, and its cart loop's 2.6 x + 6.4 v plus an integral.
Its strong cart terms replace the integral: a reference error δ parks the cart at about 5 δ (26 mm for 2 counts)
instead of 24 δ for the P cart loop. It survives slightly more runs than the PID, at about 1.7× the control effort and
twice the command changes. Without ω, evolution has to build a velocity estimate from 100 Hz quantised angles. One seed
in three found a usable 2-neuron recurrent network, and it is far weaker.

## 4. Replay of logged rig runs (open loop)

`./neuro_sim.sh` → `py/neuro_check.py replay NET.json logs/<run>`. The network sees the logged x, θ, v and ω of every
balancing stretch and its command is compared with the one the PID sent. This checks that the network behaves sensibly
on real sensor data. It cannot show closed-loop behaviour, because the logged trajectories were the PID's.

| network | session | stretches | corr with PID | rms command: PID / net | at its limit |
|---|---|---|---|---|---|
| 4 inputs (seed 1) | 2026-11-04T10-16-08 (swing-up, Stage 3b) | 3 × 30 s | 0.29–0.40 | 0.34–0.38 / 0.75–0.77 m/s² | ≤ 0.4 % |
| 4 inputs (seed 1) | 2026-11-03T14-32-03 (remote, Stage 3) | 48 s, 5 s, 60 s | 0.35–0.58 | 0.33–0.68 / 0.99–1.21 m/s² | ≤ 5 % |
| 3 inputs (seed 2) | 2026-11-03T14-32-03 | same | −0.01–0.23 | 0.33–0.68 / 1.57–1.72 m/s² | 23–39 % |

The 4-input network is plausible on real data: bounded, rarely saturated, about as smooth per update as the PID, and
2–3 times as aggressive (mostly its cart terms reacting to the ±100 mm the PID let the cart wander). The 3-input
recurrent network is not: its output is unrelated to the PID's and pinned at its limit a quarter of the time. Its
recurrent velocity estimate does not survive real quantisation and vibration. That is the same thin margin as section 1.

## 5. Re-evolution at the PID's effort

The first 4-input winner bought its survival with 1.7× the PID's effort. Three more runs (`--budget --delay 4
--delay-max 7 --stall-rate 0.5`) made the PID's own effort free (mean |a| 0.18 m/s², 4.9 command changes per s) and
anything above it expensive. They also trained with the remote path's delay range and late replies: the Pi holds the
last command for 1–3 ticks, 0.5 times per second. One run started from the first winner, two from empty networks.
They were tested on fresh draws: 300 runs of 30 s, `rig.TEST`, seed 7. With 0.5 late replies per second, no number moves
by more than 1 point or 2 mm.

| controller | survived 30 s | median end abs x | mean abs a | command changes |
|---|---|---|---|---|
| rig PID + cart loop (Stage 3b) | 81 % | 6 mm | 0.18 m/s² | 4.9 /s |
| first 4-input winner (no budget) | 85 % | 13 mm | 0.31 m/s² | 9.0 /s |
| budget, started from it | 85 % | 29–31 mm | 0.18 m/s² | 4.5 /s |
| budget, from scratch, seed 2 | 83 % | 16 mm | 0.16 m/s² | 3.7 /s |
| budget, from scratch, seed 3 | 81 % | 15–16 mm | 0.15 m/s² | 3.8 /s |

At the PID's effort, evolution finds the PID again. All three winners are one neuron with almost no recurrence; the
seeded one keeps a −0.21 self-loop. In physical units they are a ≈ 43–64 θ + 5.5 ω + 6–9 x + 7–8 v. Compare the PID's
63 θ + 6.5 ω, and its cart loop's 2.6 x + 6.4 v plus an integral. On the same draws they survive as often as the PID,
0–4 points more. With no integral, a reference error parks the cart 15–31 mm from centre instead of 6 mm.

Replayed on the logged rig runs, the seeded winner now acts like the PID. On the three 30 s balances of
2026-11-04T10-16-08 its commands correlate 0.67–0.72 with the PID's (first winner: 0.29–0.40), at the same rms
(0.35–0.39 against 0.34–0.38 m/s²). It changes slightly less per update and is almost never at its limit (0.1 %).
On the 2026-11-03 remote session the correlation is 0.48–0.73.

## 6. Headroom: would more computing on the PC help?

`py/headroom_study.py` takes every start the PID or the network loses on the same 300 test draws and asks whether any
controller could catch it within the rig's limits. Linear programs decide feasibility over 4 s, ending at rest, with the
command delay respected. Two versions:
- relaxed: no margin, the full 22°, and Coulomb friction chosen in the controller's favour. A "no" here is final.
- robust: 1 cm of slack beyond the rig's 1 cm margin, staying within 18°.
An oracle MPC (true state and plant, re-planning every 40 ms) then tries each start in the nonlinear model.
The oracle holds all 40 sampled starts that both controllers keep, and the 7 slow drifts that only the PID loses.

| the 45 starts both controllers lose | starts |
|---|---|
| no acceleration profile catches them (2 m/s², 0.3 m/s, ±0.15 m with braking distance, 8–14 ms delay) | 22 |
| catchable only with zero slack: the cart runs at v_max and grazes the braking boundary | 20 |
| catchable with 1 cm of slack | 3 (the oracle MPC holds 1) |

The failures sort by the divergent lean at engage, ξ₀ = θ₀ + θ̇₀/ω₀. Below 4° both controllers keep 98–100 %; at
4–6° about half; at 6–8° none. After a catch the cart moves at about v₀ + (g/ω₀) ξ₀. That is the swing-up gate's
predicted speed, and for 6–8° it is 0.21–0.28 m/s, inside the v_max taper. The best of PID, network and oracle keeps
256 of 300 (85 %); the network alone keeps 254. **At today's limits, more computing buys at most about 1 point.**

The binding constraints are the limits themselves. Here are the same 45 starts at the config's own priors for after
Stage 0.9:

| a_max | v_max | catchable with 1 cm slack | catchable at all |
|---|---|---|---|
| 2.0 m/s² (balance mode now) | 0.3 m/s (now) | 3 | 23 |
| 2.0 m/s² | 0.5 m/s | 13 | 27 |
| 3.0 m/s² | 0.3 m/s | 21 | 28 |
| 3.0 m/s² (`ctrl.a_max` prior) | 0.5 m/s (`ctrl.v_max` prior) | 28 | 38 |

At 3 m/s² and 0.5 m/s, 28 of the 45 become robustly catchable: up to about 94 % on these draws, against 86 % for the
best possible controller at today's limits. Stage 0.9 has to show whether the motor gets there without losing steps,
without growing the 28 Hz mode and without blowing the 1 A VMOT fuse. The PID would then be retuned, or the network
re-evolved, for the new limits.

## 7. Where this stands

- Steps 1–4 of the explainer's section 8 are done in simulation, now also at the PID's effort and the remote path's
  timing.
- Result: on this rig, with the Pi's velocity estimate, evolution converges on the tuned PID + cart loop without its
  integral. It does not beat it: survival is equal within noise and centring is worse. Pasemann's recurrent, angle-only
  design is fragile here (sections 3–4).
- Headroom (section 6): the remaining failures come from the actuator limits, not the controller. Even an oracle MPC
  adds about 1 point. Raising a_max and v_max after Stage 0.9 is the lever (about +9 points), not computing power.
- A rig run would therefore demonstrate an evolved controller, not improve balancing. The candidate is
  `py/neuro/nets/rig4_budget_seeded.json`, the closest to the PID on replay. It would need its own PC program and
  launcher script beside `remote.sh` (not `swingup.sh`), a test against `pendctl --mode remote-sim`, and an explicit
  go for each run. None of that is built.
