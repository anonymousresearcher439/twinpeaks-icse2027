================================================================================
 Living the Twin Peaks — Replication Scripts (RQ1 and RQ2)
================================================================================

This package contains the scripts used to produce the quantitative results in
Section IV of the paper:

  RQ1  Model stability      -> Fig. 3  (stability-retention Pareto fronts)
  RQ2  Incremental evolution -> Table II (evolved model M_n vs. snapshot S_n)

RQ3 (expert assessment) was an interview study and has no scripts.

The scripts are configured for one example dataset, Dronology. Other
datasets are run the same way after adding a config entry for them (see
"Adding a dataset" below). The dataset snapshots themselves (all versions
of all systems) are in the datasets/ folder of this replication package.


--------------------------------------------------------------------------------
 1. Contents
--------------------------------------------------------------------------------

  README.txt
  requirements.txt       Python dependencies
  .env.example           Template for the Anthropic API key
  KB-iso.json            ISO/IEC 25010 knowledge base used to tag ASCs
                         with quality characteristics

  core/                  Twin Peaks pipeline (shared by RQ1 and RQ2)
    analyze_architecture.py     Build a model from a full source snapshot
    analyze_delta_discovery.py  Evolution step 1: discover ASCs/DDs in the
                                files changed between two versions
    analyze_delta_alignment.py  Evolution step 2: align discoveries with the
                                prior model (retain / add / remove)
    merge_voters.py             Merge k voter models into a consensus model
                                (clustering, quorum voting, synthesis)
    twin_peaks_postprocess.py   Shared library (LLM calls, chunking, voting)
    compute_v3_metrics.py       Model-vs-model comparison: embedding matching
                                (all-mpnet-base-v2, Hungarian, theta = 0.6)

  RQ1/
    rq1_config.json         Dataset source, voter pool size, max k
    generate_voters.sh      Step 1: build the pool of independent voters
    run_RQ1_experiment.py   Step 2: k x q sweep over ensembles
    compute_stability.py    Step 3: pairwise stability across trials
    plot_pareto.py          Step 4: Pareto-front figure (Fig. 3)

  RQ2/
    rq2_config.json                 Dataset versions, seed model, voting
    versions/dronology.json         Version trajectory V0..V4 for Dronology
    run_RQ2_full.py                 Step 1: evolve M_0 -> M_n, build S_n, compare
    compute_RQ2_metrics_no_helps.py Step 2: Table II metrics
    run_snapshots.py                Timing: build a snapshot of every version
    report_timing.py                Timing: print snapshot and evolution times

  results/
    timing-dronology/       Measured times for Dronology: TIMING.txt plus
                            the raw per-step timing and token files


Terminology: the paper uses ASC (Architecturally Significant Concern) and
DD (Design Decision). In the code and JSON files these are nodes with
kind "ag" and kind "ad" respectively, and some output labels say ASG / AD.


--------------------------------------------------------------------------------
 2. Setup
--------------------------------------------------------------------------------

Requirements: Linux or macOS, Python 3.10+, an Anthropic API key. A GPU is
optional (it only speeds up the embedding step).

  python3 -m venv venv
  source venv/bin/activate
  pip install -r requirements.txt

  cp .env.example .env          # then put your key in .env

The API key is read from a .env file in this folder or any parent folder,
or from the ANTHROPIC_API_KEY environment variable.

Dataset layout
--------------
The scripts read the source snapshots from the datasets/ folder next to
this scripts/ folder, one subfolder per system and one per version:

  datasets/
    dronology/
      v0/     V0  DATASET_V001          2018-05-11   (RQ1 baseline)
      v1/     V1  integration@cc927a2   2018-06-19
      v2/     V2  integration@902769b   2018-07-12
      v3/     V3  ICSE_2019_DATA_V1     2018-11-13
      v4/     V4  integration           2019-02-12
    px4/          v0 .. v4
    ardupilot/    v0 .. v4
    aerostack2/   v0 .. v4
    dronboard/    v0 .. v4

The source paths in RQ1/rq1_config.json and RQ2/versions/*.json are
written as $TP_DATASETS/<system>/v<n>. TP_DATASETS defaults to that
datasets/ folder; to keep the snapshots somewhere else, set it:

  export TP_DATASETS=/path/to/datasets

All LLM calls use claude-sonnet-4-6 (set in core/twin_peaks_postprocess.py).
The first run downloads the all-mpnet-base-v2 embedding model (~420 MB).


--------------------------------------------------------------------------------
 3. Checking the setup without API calls
--------------------------------------------------------------------------------

These commands make no API calls and cost nothing:

  cd RQ1
  python3 run_RQ1_experiment.py --dataset dronology --smoke-test
      # prints the voter-to-trial assignment for every k (seeded, so it
      # reproduces the assignment used in the paper)
  python3 run_RQ1_experiment.py --dataset dronology --dry-run

  cd ../RQ2
  python3 run_RQ2_full.py --dataset dronology --dry-run
      # prints every pipeline command that would run, with resolved paths


--------------------------------------------------------------------------------
 4. RQ1 — Model stability (Fig. 3)
--------------------------------------------------------------------------------

For one version V0 of a system, RQ1 builds ensembles of k = 1..5 voters,
3 independent trials per k, with no voter reused across trials. For every
quorum q = 1..k it measures stability (mean pairwise ASC coverage across
the 3 trials) and retention (|ASC(k,q)| / |ASC(k,1)|).

All commands run from RQ1/.

  Step 1 — Generate the voter pool (15 independent single-run models):

    bash generate_voters.sh dronology
      -> voters/dronology/run1/V0.json ... run15/V0.json

  Step 2 — Run the ensemble sweep for k = 1..5:

    python3 run_RQ1_experiment.py --dataset dronology
      -> experiments/dronology/v<k>/trial<1-3>/t<q>/V0-ensemble.json
         experiments/dronology/v<k>/trial<1-3>/final/V0-ensemble.json
         experiments/dronology/v<k>/threshold_sweep.json
    (use --level <k> to run a single ensemble size)

  Step 3 — Compute pairwise stability:

    python3 compute_stability.py --dataset dronology
      -> experiments/dronology/v<k>/stability/summary.json
         experiments/stability_summary.csv

  Step 4 — Plot the Pareto front:

    python3 plot_pareto.py dronology     # one dataset
    python3 plot_pareto.py --grid        # all datasets present (Fig. 3)
      -> experiments/dronology/pareto_front.png
         experiments/pareto_grid.pdf

The k = 5 trial-2 ensemble (experiments/dronology/v5/trial2/final/
V0-ensemble.json) is the baseline model M_0 used by RQ2.


--------------------------------------------------------------------------------
 5. RQ2 — Incremental evolution (Table II)
--------------------------------------------------------------------------------

Starting from M_0, RQ2 evolves the model through each version in
RQ2/versions/<dataset>.json (V0 -> V1 -> ... -> Vn) using the two-step
delta pipeline (discover, then align). It then builds a snapshot model S_n
from scratch on Vn and compares M_n against S_n. This is repeated for 3
trials.

Note on the trials: as in the experiments reported in the paper, all three
trials start from the same M_0 (the RQ1 k = 5 trial-2 ensemble). The
variation across trials therefore comes from the non-deterministic LLM
steps (delta discovery, alignment and the snapshot S_n), not from different
starting models. To give each trial its own M_0 instead, write trial{trial}
in place of trial2 in the v0_json path in rq2_config.json. Trial n then
starts from the RQ1 k = 5 trial-n ensemble.

Requires RQ1 to have been run first (RQ2 reads M_0 from RQ1/experiments/).

All commands run from RQ2/.

  Step 1 — Evolve, build snapshots and compare:

    python3 run_RQ2_full.py --dataset dronology
      -> experiments/dronology/trial<n>/v<i>/V<i>-aligned.json   (M_i)
         experiments/dronology/trial<n>/scratch/V4-scratch.json  (S_n)
         experiments/dronology/trial<n>/comparison.json
         experiments/dronology/rq2_summary.json
    (use --trial <n> to run a single trial)

  Step 2 — Table II metrics (helps edges excluded):

    python3 compute_RQ2_metrics_no_helps.py
    python3 compute_RQ2_metrics_no_helps.py --latex

The settings used in the paper are in rq2_config.json: 4 discovery runs per
delta step with a vote threshold of 2.

  Timing — snapshot vs. evolution:

    run_RQ2_full.py records the wall-clock time of every evolution step
    (V<i>-timing.json: discover_s, align_s) and of each final snapshot
    (V<n>-scratch-timing.json). To time a from-scratch snapshot of every
    version, and then print both:

    python3 run_snapshots.py --dataset dronology
      -> experiments/dronology/snapshots/V<i>-snapshot.json
         experiments/dronology/snapshots/V<i>-snapshot-timing.json
    python3 report_timing.py --dataset dronology

    Times are wall-clock and dominated by LLM API latency, so they vary
    with API load. See section 7 for measured values.


--------------------------------------------------------------------------------
 6. Adding a dataset
--------------------------------------------------------------------------------

  RQ1: add an entry to RQ1/rq1_config.json:

    "px4": {
      "source":   "$TP_DATASETS/px4/v0",
      "pool_dir": "voters/px4",
      "n_voters": 15,
      "max_v":    5
    }

  RQ2: create RQ2/versions/<dataset>.json listing the versions in order
  (each entry needs "name" = V0, V1, ... and "source"), then add an entry to
  RQ2/rq2_config.json pointing to it and to the dataset's RQ1 model:

    "px4": {
      "versions_json":  "versions/px4.json",
      "v0_json":        "../RQ1/experiments/px4/v5/trial2/final/V0-ensemble.json",
      "discovery_runs": 4,
      "voting_t":       2
    }


--------------------------------------------------------------------------------
 7. Cost, runtime and reproducibility
--------------------------------------------------------------------------------

Approximate API cost for Dronology at claude-sonnet-4-6 prices
($3 / $15 per million input / output tokens), taken from our runs:

  RQ1 step 1  15 voters, ~160K in / ~25K out tokens each
              (~$0.85 and ~7 minutes per voter)                 ~ $13
  RQ1 step 2  ensembles k = 2..5, 3 trials each
              (k = 1 needs no merging)                          ~ $24
  RQ2 step 1  3 trials; per trial ~1.5M in / ~0.28M out for the
              evolution V0..V4, plus a ~$2 snapshot of V4       ~ $30
  All other steps run locally (no API calls).

Larger systems cost proportionally more (roughly 130K-310K input tokens
per voter across the five systems in the paper).

Measured timing (Dronology, paper settings, one run, 2026-09-25; produced
with run_snapshots.py, run_RQ2_full.py and report_timing.py; details and
raw data in results/timing-dronology/). Wall-clock
minutes, sequential API calls, claude-sonnet-4-6:

  Build a snapshot from scratch (1 extraction run + post-processing)

    Version   KLOC   Minutes   Cost
    V0        17.6     8.3    $0.91
    V1        21.8    14.2    $1.33
    V2        25.9    16.8    $1.51
    V3        26.9    16.4    $1.57
    V4        27.7    16.9    $1.59   (mean of 2 builds: 16.3, 17.4)

  Evolve the model to the next version (4 discovery runs + vote + align)

    Step     Discover   Align   Total   Cost
    V0->V1     13.6      2.2    15.8   $1.81
    V1->V2     18.6      3.1    21.7   $2.71
    V2->V3     19.6      2.9    22.5   $2.71
    V3->V4     16.1      3.0    19.1   $1.59

  Within a discovery step, each of the 4 extraction runs on the changed
  files took 2.3-3.7 minutes, and voting + synthesis took 3.8-5.8 minutes.
  The runs are independent, so running them in parallel would bring an
  evolution step down to roughly 9-13 minutes. The scripts run them one
  after another.

Resuming: every step skips outputs that already exist, so an interrupted
run can be restarted with the same command. Delete an output file (or use
--force where available) to recompute it. Alignment caches the raw LLM
response in V<i>-aligned-raw-alignment.json.

Reproducibility: LLM extraction is non-deterministic, so a rerun will not
produce identical models. The evaluation is designed for this: results are
aggregated over independent voters and trials, and the analysis steps
(stability, coverage, cosine similarity) are deterministic given the
models.

The only random seed in the scripts is SEED = 42 in run_RQ1_experiment.py.
It controls which voters from the pool are assigned to which trial for
each ensemble size k, so the same voters land in the same trials on every
run. It does not affect the LLM calls.
