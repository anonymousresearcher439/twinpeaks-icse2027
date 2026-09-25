# Timing — Snapshot vs. Evolution

[Back to README](readme.md)

How long it takes to (1) build a Twin Peaks model of a version from scratch, and
(2) evolve the model from one version to the next. Measured on Dronology
(2026-09-25) with `claude-sonnet-4-6`, sequential API calls, wall-clock time
around each pipeline call.

- **Snapshot:** 1 extraction run + post-processing (the setting used for the snapshot model *S<sub>n</sub>*).
- **Evolution:** 4 discovery runs on the changed files, vote threshold 2, then alignment (the RQ2 settings in the paper).

## (1) Build a snapshot from scratch

| Version | Files | KLOC | Minutes | Input tokens | Output tokens | Cost |
|---|---:|---:|---:|---:|---:|---:|
| V0 | 261 | 17.6 | 8.3 | 161,672 | 28,154 | $0.91 |
| V1 | 302 | 21.8 | 14.2 | 191,464 | 50,326 | $1.33 |
| V2 | 357 | 25.9 | 16.8 | 222,821 | 56,310 | $1.51 |
| V3 | 392 | 26.9 | 16.4 | 248,595 | 55,256 | $1.57 |
| V4 | 405 | 27.7 | 16.9 | 253,453 | 55,531 | $1.59 |

V4 is the mean of two builds (16.3 and 17.4 min).

## (2) Evolve the model to the next version

| Step | Discover (min) | Align (min) | Total (min) | Input tokens | Output tokens | Cost |
|---|---:|---:|---:|---:|---:|---:|
| V0 → V1 | 13.6 | 2.2 | 15.8 | 318,076 | 56,726 | $1.81 |
| V1 → V2 | 18.6 | 3.1 | 21.7 | 511,118 | 78,261 | $2.71 |
| V2 → V3 | 19.6 | 2.9 | 22.5 | 495,818 | 81,172 | $2.71 |
| V3 → V4 | 16.1 | 3.0 | 19.1 | 180,401 | 70,065 | $1.59 |

Within a discovery step, each of the 4 extraction runs on the changed files took
2.3–3.7 min, and voting + synthesis took 3.8–5.8 min. The 4 runs are independent;
running them in parallel would bring an evolution step to roughly 9–13 min. The
scripts run them one after another.

Costs use $3 / $15 per million input / output tokens. Times depend on API load
and will vary between runs.

## Reproducing

From [`scripts/RQ2/`](scripts/RQ2/):

```
python3 run_snapshots.py --dataset dronology            # time a snapshot of every version
python3 run_RQ2_full.py  --dataset dronology --trial 1  # time each evolution step
python3 report_timing.py --dataset dronology            # print the tables
```

Raw per-step timing and token files:
[`scripts/results/timing-dronology/`](scripts/results/timing-dronology/).
