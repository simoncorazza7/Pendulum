# Evolved networks kept in the repo

The winning network of every ENS³ run of 2026-11-19 (`docs/neuro_sim.md`). The full run directories (`log.csv`,
every saved elite, `summary.json`) stayed in the git-ignored `logs/`; each file here names its run in
`meta.source_run`. Format: `neuro.net.Genome` (`Genome.load(path)`): inputs, class, hidden units, biases, connections
as `[to, from, weight]`, and `meta` with the run's settings (`net_every`, delays, late replies, budget).

Rig-model results are on the same 300 start states and plants (`rig.TEST`, seed 7), 30 s each:
`./neuro_sim.sh rig <file> --runs 300 --t 30 --seed 7`. On those draws the rig's PID with the Stage 3b cart loop keeps
81 %, ends 6 mm from centre and uses 0.18 m/s² mean |a|.

| file | inputs | how it was evolved | result |
|---|---|---|---|
| `bench_4t.json` | x, θ, ẋ, θ̇ (Pasemann's benchmark) | 1000 generations from empty networks | 1388 of 1444 grid squares (the paper's w¹: 1390); w¹'s structure |
| `bench_2t.json` | x, θ | 3000 generations from empty networks | 1398 of 1444 (w⁴: 1396); w⁴'s architecture |
| `rig3_s1.json`, `rig3_s2.json`, `rig3_s3.json` | x, θ, v (θ̇ by recurrence) | rig model, 600 generations | kept 4 %, 52 %, 1 %; `rig3_s2` is not sane on replayed rig logs |
| `rig4_s1.json`, `rig4_s2.json`, `rig4_s3.json` | x, θ, v, the Pi's ω | rig model, 600 generations, no effort budget | kept 85 % each, at 0.29–0.35 m/s² (1.7× the PID) |
| `rig4_budget_seeded.json` | x, θ, v, ω | 300 generations from `rig4_s1`, PID effort budget, delays 4–7 ticks, late replies | kept 85 %, ends 29–31 mm off centre, 0.18 m/s²; the rig candidate and `py/headroom_study.py`'s default |
| `rig4_budget_s2.json`, `rig4_budget_s3.json` | x, θ, v, ω | 400 generations from empty networks, same budget and timing | kept 83 % / 81 %, 16 mm, 0.15–0.16 m/s² |

None of these has run on the rig.
