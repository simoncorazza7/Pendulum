# Upright-offset estimate, carried between catches (simulation prototype, 2026-11-19/20)

Status: simulation, log replay, and a port into the Pi runtime (§6), switched off by default. The port is tested on
the PC only: it has not been built or run on the Pi. It is a separate method with its own scripts: `./offset_sim.sh`
(PC) and `./offset.sh` (rig), `py/offset/`, `py/offset_check.py`, `py/offset_study.py`, `py/test_offset.py`,
`src/core/offset_est.*`, `src/runtime/offset_params.h`. `demo.sh`, `remote.sh`, `swingup.sh`, `balance.cpp` and
`swingup.cpp` are unchanged.

**Result.**
- **The method:** estimate the upright reference error online from the drift identity during every balance, and set
  it as the trim when the next catch engages, held for the catch.
- **On the logged sessions:** it halves the offset a catch starts with.
- **In simulation:**
  - the 300 rig draws go from 81 % to 85 %, the ceiling for any controller at today's limits;
  - the rig hand-overs at a 2.5-count offset go from 5 to 16 of 29.
- **What not to do:** change the trim within a catch. It fights the cart integral (77 %).
- **Rig status:** ported to the Pi runtime (`./offset.sh watch|apply`, off by default) and tested on the PC, including
  `SimHw` sessions. Not yet built or run on the Pi (§6).

```
./offset_sim.sh test                         13 checks, no logs needed
./offset_sim.sh replay [RUN ...] [--cache D]  the estimator on every logged balance session (needs logs/)
./offset_sim.sh centres RUN ...               hanging equilibrium between swing-up runs (needs logs/)
./offset_sim.sh handovers                     closed loop: the 29 rig hand-overs, Stage 3b (~15 min)
./offset_sim.sh draws                         closed loop: the 300 rig.TEST draws of docs/neuro_sim.md (~20 min)
```

## 1. The problem

The balance loop holds the measured angle θ. If the upright reference is off by δ (θ_true = θ + δ), the pendulum
really leans by δ, and holding that lean takes a steady cart acceleration g·δ (brief §3). Stage 3b's cart integral
removes it, slowly (time constant ~5 s), and it starts from zero at every catch (`docs/stage3_cascade.md`).

- 2026-11-04: δ was +1.2…+1.4 counts (the day before +0.08), and 16 of 17 Stage 3 balances ended at the +x soft limit.
- Stage 3b in simulation keeps 20 of the 29 rig hand-overs at 1.3 counts, but only 5 of 29 at 2.5 counts.

The idea: measure δ while the pendulum balances, and start the next catch with it as the trim.

## 2. The estimator (`py/offset/identity.py`)

It is the drift identity of `docs/stage1_drift.md`, the one `py/stage1_report.py` fits offline, made online. Along any
balance, whatever the controller does:

    r(t) = (v − v_s) − g ∫θ dt + L_eq (θ̇ − θ̇_s) + L_eq (b/J)(θ − θ_s)  =  g·δ·(t − t_s) + noise

Here _s marks the start of a stretch and L_eq = g/ω₀². It is the linearised model integrated once, so it needs no
acceleration measurement and no model of the controller. The offline version predicted the 11-04 parking offset and
set the 10-14 trim (residual −0.12 counts).

Online, one tick at a time:

| step | rule | value |
|---|---|---|
| a stretch starts | t_settle after engage (hand on the rod, catch transient) | 1.5 s |
| it ends | \|θ\| above θ_max, end of the balance, or a push | 3° |
| a push | the residual's 0.25 s mean moves more than `step` within `lag`, beyond the fitted slope | 0.03 m/s in 0.3 s |
| samples are committed | `lag` late, so a push, a fall or an abort discards the samples that saw it | 0.3 s |
| a stretch counts | with at least `min_stretch` committed | 1 s |
| the estimate | length-weighted mean of all stretch slopes so far (the pool), / g, clamped | ±5 counts |
| it is used | at engage, once `W_USE` seconds of quiet balance stand behind it; held for the catch | 3 s |

The pool persists between balances; that is the carry. A prior (e.g. from an earlier session) enters as a slope
with its own weight in seconds. Stretches are weighted by length, not by the T³ of a white-noise line fit: the
residual's noise is correlated (the 14 Hz rate estimate, pivot friction), and on the 2026-11-02 pivot the quiet
stretches of a single 60 s balance scatter by ~0.7 counts (sd).

Two properties the checks pin down (`py/test_offset.py`):
- **It measures what the cart feels.** With the exact angle it recovers a set offset to 0.01 count. With the 2400-count
  encoder it equals the batch fit of the same residual, which includes the quantisation bias: the limit cycle dithers
  across the count boundary nearest upright, worth up to half a count (+1.3 set, +1.61 felt). That bias is real drift,
  so the trim should include it.
- **Observability.** In the measured frame (θ, θ̇, d = ω₀²δ) the observability matrix from θ alone has determinant 1:
  d is the part of θ̈ that the model and the known cart acceleration do not explain. In the true-angle frame it is ω₀²
  (gravity ties a steady lean to a steady acceleration). A stepper knows the cart acceleration, so δ is observable.

### Why not a fast observer

`py/offset/observer.py` is the first attempt: a fixed-gain observer for (θ, θ̇, d) with the measured cart
acceleration Δv/Ts as its input, poles placed at 6 Hz for angle and rate and 1/τ_d = 1 s⁻¹ for the offset. On the
old pivot it matched the drift identity to 0.1–0.2 count (10-14 sessions). On the 2026-11-02 pivot its estimate swings
by about ±2 counts within one balance. Example: 11-04 08-58-08 run 1, where the Stage 3 P loop swayed ±1.5° and
±100–240 mm/s. Over 1 s windows the offset evidence ranges from −2.5 to +5 counts, while the identity's cumulative
fit holds +1.23…+1.26 from 3 s on. The offset is a constant and needs a long memory, so the observer is not used for
it. It stays in the package for the open rate-estimate question (§7).

## 3. Logged sessions (open loop)

`./offset_sim.sh replay`: every balance of every logged balance session (hand catches and swing-up hand-overs, old
and new pivot; 18 sessions have at least one quiet stretch) goes through the estimator, its pool carried from
balance to balance. The estimator only watches; the logged runs were balanced by the rig's own controller. Each balance with a quiet stretch
has its own drift-identity offset (the ground truth). A catch today starts with that offset uncorrected; with the
carry it starts with (own − carried).

| balances | n | mean \|offset\| at engage, without / with carry (counts) | median | worse with carry |
|---|---|---|---|---|
| all with an own offset | 44 | 0.88 / 0.69 | 0.73 / 0.39 | 8 |
| those with a carried estimate (≥ 3 s behind it) | 21 | **0.81 / 0.43** | **0.59 / 0.18** | 8 |
| … hand catches | 15 | 0.73 / 0.36 | 0.45 / 0.18 | 6 |
| … swing-up hand-overs | 6 | 1.01 / 0.59 | 1.01 / 0.49 | 2 |

- The carry halves the offset a catch starts with. The large-offset sessions gain most: 10-12 15-30-52 (+2.0…+2.3,
  untrimmed) and 11-04 08-58-08 (+1.22, +1.21, +1.26; carried +1.44).
- Where it is worse, the offset was already small. In the 10-14 sessions the manual trim −2.1 had removed most of it,
  and the carry adds 0.1–0.3 count of noise.
- **Between-balance scatter on the 2026-11-02 pivot.** Hand-catch sessions agree with themselves: 11-03 14-32-03
  gave +0.08 and −0.10; 11-04 08-58-08 gave +1.22, +1.21 and +1.26. Swing-up sessions do not. Two 30 s balances of
  11-03 15-32-17 gave +0.10 and +1.63, and of 11-04 10-16-08 −1.42 and +0.22. That scatter is the floor of any carry.
  - A Coulomb term in the identity (0.168 rad/s², tanh at 0.02–0.15 rad/s) does not reduce it.
  - The count reference does not move between runs. `./offset_sim.sh centres` fits the hanging equilibrium after
    every swing-up run: within ±0.2 count over each session, so no edges are lost in the fast rotation.
  - The cause is open. A candidate is stiction during the long, steady leans that bring a fast hand-over back to the
    centre.

## 4. Closed loop (simulation)

`py/offset_study.py` adds the trim to the rig's own Stage 3b cascade (`cascade_step` gains). With the trim off it
reproduces `py/cascade_pi_study.py` and the 81 % of `docs/neuro_sim.md` exactly. The variants:

| variant | trim |
|---|---|
| off | none (the rig today) |
| online | the estimator from nothing, applied within the catch it is learning from (after 3 s, ≤ 1 count/s) |
| online-bl | the same, each change handed to the cart integral bumplessly (x_int −= g·Δtrim/ki) |
| carry | the estimate learnt in the earlier catches, set at engage and held; learning goes on for the next catch |
| carry-always | carried, and also updated within the catch |
| integral | the quick version: a catch that lasted ≥ 10 s passes its settled cart-integral lean (ki·x_int/g) to the next |
| prior | [draws] the true offset plus a normal error (sd 0.5 or 1.0 count), set at engage and held |
| prior-always | [draws] the same prior (weight 20 s), also updated within the catch |
| ideal | the true offset from the start (upper bound) |

### The 29 rig hand-overs as one session (`./offset_sim.sh handovers`)

Stage 3b, the 11-02 plant, BD+LP2 14 Hz, quantisation, Axis cap and soft limits as `py/cascade_pi_study.py`. The
cases run in their logged order, so a carried estimate exists only after a balance has given the estimator 3 s of
quiet balance.

| true offset | off | online | carry | carry-always | integral | ideal |
|---|---|---|---|---|---|---|
| +1.3 counts | 20/29 | 20/29 | **22/29** | 22/29 | 22/29 | 22/29 |
| +2.5 counts | 5/29 | 5/29 | **16/29** | 16/29 | 16/29 | 22/29 |

- **online changes nothing.** Every loss comes within 0.2–5.8 s, before the estimate exists (1.5 s settle + 3 s of
  quiet balance).
- **carry reaches the ideal at 1.3 counts.** The 7 losses left are the catch kick within 1.2 s, which no trim prevents
  (the hand-over speed gate of `docs/stage3_cascade.md` addresses those).
- **At 2.5 counts carry misses 6 that ideal keeps.** They fall at 1.3–2.5 s, at the same moments as with no trim:
  no balance before them had lasted long enough to learn from, so they started untrimmed. From then on carry loses
  only what ideal loses. Its trim ends at +2.54 counts (median), within 0.05 of the set offset (+1.42 at 1.3 counts:
  the quantisation bias of §2).
- **integral (the quick version) does as well here.** It needs a catch of ≥ 10 s to learn from, where the identity
  needs ~5 s of balance (1.5 s settle + 3 s), and it exists only where the cart integral runs.

### 300 rig.TEST draws, each a first catch (`./offset_sim.sh draws`)

The 300 draws of `docs/neuro_sim.md` (seed 7): offsets to ±2 counts, ω₀ ±3 %, b/J 0.03–0.08, Coulomb 0.10–0.25,
delays of 4–7 ticks. Each draw is a separate first catch, so a carried estimate is modelled as a prior: the true
offset plus a normal error.

| trim | survived 30 s | lost: \|offset\| > 1 count (of 136) | lost: \|offset\| ≤ 1 (of 164) | median end \|x\| |
|---|---|---|---|---|
| off | 81 % (244) | 27 | 29 | 6 mm |
| online | 77 % (230) | 29 | 41 | 7 mm |
| online-bl | 81 % (244) | 27 | 29 | 5 mm |
| prior, error sd 0.5 count, set at engage | **85 % (254)** | 17 | 29 | 6 mm |
| prior, sd 0.5, also updated within the catch | 84 % (252) | 19 | 29 | 7 mm |
| prior, error sd 1.0 count, set at engage | 83 % (248) | 19 | 33 | 7 mm |
| ideal | 85 % (254) | 16 | 30 | 5 mm |

- **Changing the trim within a catch does not help.**
  - Handed to the cart integral bumplessly (online-bl), it changes nothing, run for run.
  - Without that hand-off (online), it loses 12 more runs, 10 of them with offsets of 1 count or less. The cart
    integral has already corrected part of the offset by then, so the trim change comes on top of that correction:
    corrected twice.
- **A trim set at engage from earlier balances reaches the ideal.**
  - With a 0.5-count error it reaches 85 %. On the rig logs the carried estimate was off by 0.43 count on average (§3).
  - With a 1-count error it still reaches 83 %.
- **85 % is the ceiling.** `docs/neuro_sim.md` §6 found it for any controller at today's limits on these draws: the
  best of PID, evolved network and oracle MPC keeps 256 of 300.
  - The evolved network kept 254, partly because its strong cart terms park an offset closer to the centre.
  - The trim gives the rig's own PID + cart loop the same 254.

## 5. What this means

**Learn always, apply at engage.** The estimator runs during every balance. The next catch starts with
θ_ref = −δ̂ once at least 3 s of quiet balance stand behind the estimate, and that trim is held for the whole catch.
Within a catch, the offset is left to the Stage 3b cart integral.

- **What it buys, in simulation at today's limits:**
  - rig.TEST draws: 81 % → 85 %, the ceiling of `docs/neuro_sim.md` §6;
  - rig hand-overs at 2.5 counts: 5 → 16 of 29 (ideal 22, the gap being the catches before the first long
    balance); at 1.3 counts: 20 → 22 (the ideal).
  - On the logs, the offset a catch starts with halves (0.81 → 0.43 counts).
- **What it costs:** no change to the control law, only a trim at engage. The risk is a wrong carried estimate:
  - with a 1-count error the draws still gain 2 points;
  - where a good manual trim had already removed the offset, the carry adds 0.1–0.3 count (§3).
- **The quick version, the cart integral's settled lean, does as well in the hand-over simulation.** The drift
  identity is still the better choice for the rig:
  - it learns from about 5 s of balance, where the quick version needs a catch of at least 10 s;
  - it does not depend on the controller, so it works the same in Stage 1, Stage 3b, the swing-up balance and on the
    remote PC;
  - it rejects pushes;
  - it can be checked against any log.
- **It does not replace a good calibration.** The offset changes at every program start (the start-up fit, §6), so
  the pool starts empty in each session. The first catch of a session stays untrimmed until a balance has run about 5 s.

## 6. The Pi runtime (ported 2026-11-20; tested on the PC, not yet on the Pi)

**The estimator.** `core::DriftIdentityEstimator` (`src/core/offset_est.*`) mirrors `py/offset/identity.py` operation
for operation. The golden test replays 12 500 ticks from `py/offset_golden.py` (a push, an excursion past θ_max, a
balance end, a carry into a second balance, a prior), and the worst relative difference is 0: bit-identical. Its
buffers are fixed rings sized once at construction, so `step()` never allocates in the real-time loop.

**The session** (`src/runtime/session.*`) owns one estimator for the whole program run:
- it feeds it during every balance: a hand catch while ENGAGED, a swing-up run while in BALANCE; the clock starts at
  engage or hand-over;
- with `apply`, it sets the trim where each catch's runtime is created (`startBalance`, the start of each swing-up
  run): θ_ref = config − δ̂, once `offset.w_use` seconds stand behind the estimate. The trim is held for that catch;
- `balance.cpp` and `swingup.cpp` are unchanged.

**Configuration:** `offset.*`, read by `src/runtime/offset_params.h`. Every key is optional; the defaults used are
written to each run's `config_used.conf`.

| key | unit | default | |
|---|---|---|---|
| `offset.mode` | | off | `off`, `watch` (estimate and log only), `apply` (also trim each catch) |
| `offset.w_use` | s | 3 | quiet balance before `apply` uses the estimate |
| `offset.t_settle` | s | 1.5 | after engage or hand-over before a stretch may start |
| `offset.theta_max_deg` | deg | 3 | \|θ\| that ends a stretch |
| `offset.lag` | s | 0.3 | commit delay = step-detection window |
| `offset.smooth` | s | 0.25 | moving mean for the step detector |
| `offset.step` | m/s | 0.03 | residual step that marks a push |
| `offset.min_stretch` | s | 1 | committed before a stretch counts |
| `offset.delta_max_counts` | counts | 5 | clamp of the estimate |

`apply` is refused together with `balance.remote`. The PC computes the command from its own PARAMS, which carry
θ_ref, are sent once and are checksummed, so a per-catch trim would be a protocol change. `watch` works remotely.

**Logs:**
- `data.csv` gains `offset_delta` (rad), `offset_weight` (s) and `offset_trim` (rad), appended as the last columns;
- `meta.json` gains `offset_mode`, `offset_delta_counts`, `offset_weight_s` and `offset_stretches`;
- after every run the session prints the estimate, its weight and the trim that run used.

`./offset_sim.sh replay <run>` puts the Pi's logged estimate next to the offline replay of the same balances. They
are the same algorithm on the same logged inputs, so they should agree to rounding. This path has not seen a real
log yet.

**Launcher `./offset.sh`** (on the Pi, a hardware action, its own script per the project rule):
- `./offset.sh watch [--swingup | --remote | --swingup --remote]` or `./offset.sh apply [--swingup]`;
- it runs `./demo.sh`, `./swingup.sh` or `./remote.sh` with `--set offset.mode=…`, so their checklists and limits apply
  unchanged.

**Tests** (`bash tests/unit/run_pc.sh`: 113 of 113, 7 of them new in `tests/unit/test_offset_est.cpp`):
- the golden test;
- parameter validation and `offset.*` parsing, including `apply` + remote refused;
- a prior alone is the estimate;
- **watch, `SimHw` session with a 1-count reference error:** every command bit-identical to `off`; estimate +0.68
  count from 13 s, which is the quantisation bias of §2 at work;
- **apply, hand catches:** the second catch starts with the learnt trim, held for the whole catch. From 4 s to 15 s
  after engage, its cart stays within 4.3 mm; untrimmed it wandered to 31.2 mm while the cart integral learnt the
  offset;
- **apply, swing-up:** run 1 balances 30 s and learns +1.18 counts; run 2 starts with exactly that trim.

**Next on the rig** (each run needs a go; the statement before each run as `CLAUDE.md` requires):
1. **Pi build and tests.** Push to the Pi (`git push pi control-dev`, whose tree must be clean), then `make test` there.
   That builds nothing that moves and runs nothing that touches GPIO. The `gpio-mockup` session tests need `sudo
   modprobe` (ask first).
2. **Watch.** `./offset.sh watch` (hand catches) and `./offset.sh watch --swingup`: the commands are unchanged. Then
   `./offset_sim.sh replay <run>` and the drift report (`py/stage1_report.py balance`) to compare.
3. **Apply.** `./offset.sh apply`, first with hand catches, then with swing-ups.

**Not across sessions.** The offset changes between program starts (the start-up c_bottom fit), so the pool starts
empty in every program run.

Side finding, worth a look: in 3 of the 4 new-pivot swing-up sessions, the start-up c_bottom fit
differs by 0.6–0.8 count from the hanging equilibrium fitted after every run with the cart at rest (`centres`):

| session | start-up fit | after the runs |
|---|---|---|
| 11-03 10-02-18 | −0.97 | −0.87 … −1.08 |
| 11-03 15-32-17 | +0.40 | −0.26 … −0.73 |
| 11-04 09-15-27 | +1.86 | +1.13 … +1.46 |
| 11-04 10-16-08 | −0.63 | −1.06 … −1.45 |

Each such error enters δ directly at every program start. The calibration's window and decay model
(`calib.swing_sigma`, 12–18° swings on a pivot with Coulomb friction) are the first suspects.

## 7. Open

- **Rate estimate.** A model-based θ̇ (the observer, with the known cart acceleration) could have less lag than BD+LP2
  at 14 Hz. Not evaluated: it has to pass no more of the ~28 Hz drive-side mode than the 14 Hz filter does, and the
  margins need a loop broken at the actuator with the observer inside it (`pendel.linear`).
- **The swing-up scatter** of §3: until it is understood, the carry on swing-up sessions is worth about half what it
  is worth on hand catches.
- **The logs are not in git.** `replay` and `centres` need the run directories under `logs/`.
- **Remote apply.** It would need the trim in the PC's PARAMS (a protocol version) or the estimator on the PC
  (`py/remote_ctrl.py` sees θ, θ̇ and v in every sample).
- **The `offset.*` keys are not in `config/pendctl.conf` yet.** They are optional with defaults; add a commented
  section the next time that file is edited.
