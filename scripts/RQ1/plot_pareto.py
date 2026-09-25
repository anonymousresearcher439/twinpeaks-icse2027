#!/usr/bin/env python3
"""Generate Pareto front plot: stability vs mean ASCs across all (k, q) combinations.

Usage:
  python plot_pareto.py              # combined plot of all available datasets
  python plot_pareto.py --panels     # 3-panel analytical view (k-colour + q quorum)
  python plot_pareto.py --grid       # 2x3 grid: 6 Pareto panels
  python plot_pareto.py <dataset>    # single-dataset plot (original behaviour)
"""

import json
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path

HERE = Path(__file__).parent

# One colour reserved per dataset slot (up to 6 datasets).
# Shape encodes Pareto status: diamond = Pareto winner, circle = non-winner.
DATASET_COLORS = [
    "#8c564b",  # slot 0 — brown  (dronboard-early)
    "#1f77b4",  # slot 1 — blue   (dronboard)
    "#ff7f0e",  # slot 2 — orange (dronology)
    "#2ca02c",  # slot 3 — green  (px4)
    "#d62728",  # slot 4 — red    (aerostack2)
    "#9467bd",  # slot 5 — purple (ardupilot)
]

# Canonical ordering and display metadata for the 2x3 grid
DATASET_ORDER = ["dronboard-early", "dronboard", "dronology", "px4", "aerostack2", "ardupilot"]
DATASET_LABELS = {
    "dronboard-early": "DROnboard (Early)\n(1.4 KLOC)",
    "dronboard":       "DROnboard\n(9.9 KLOC)",
    "dronology":       "Dronology\n(17.6 KLOC)",
    "px4":             "PX4 Arming\n(14.2 KLOC)",
    "aerostack2":      "AeroStack2\n(18.5 KLOC)",
    "ardupilot":       "ArduPilot\n(16.1 KLOC)",
}
DATASET_LINE_COLORS = {ds: DATASET_COLORS[i] for i, ds in enumerate(DATASET_ORDER)}

K_MAX = 5  # never plot k > K_MAX (aerostack2 has k=6,7 which we exclude)

def load_points(dataset):
    pts = []
    for v in range(1, K_MAX + 1):
        sweep_file = HERE / "experiments" / dataset / f"v{v}" / "threshold_sweep.json"
        if not sweep_file.exists():
            continue
        d   = json.loads(sweep_file.read_text())
        opt = str(d["optimal_t"])
        for t, m in sorted(d["by_threshold"].items(), key=lambda x: int(x[0])):
            pts.append({
                "dataset":   dataset,
                "k":         v,
                "t":         int(t),
                "stability": m["stability"],
                "mean_ags":  m["mean_ags"],
                "f1":        m["f1"],
                "optimal":   t == opt,
            })
    return pts

def is_pareto(p, all_pts):
    for q in all_pts:
        if q is p:
            continue
        if q["stability"] >= p["stability"] and q["mean_ags"] >= p["mean_ags"]:
            if q["stability"] > p["stability"] or q["mean_ags"] > p["mean_ags"]:
                return False
    return True

def draw_pareto_step(ax, pts, color, linewidth=0.6):
    pareto_pts = sorted([p for p in pts if is_pareto(p, pts)], key=lambda x: x["mean_ags"])
    if len(pareto_pts) < 2:
        return
    px = [p["mean_ags"] for p in pareto_pts]
    py = [p["stability"] for p in pareto_pts]
    step_x, step_y = [px[0]], [py[0]]
    for i in range(1, len(px)):
        step_x += [px[i], px[i]]
        step_y += [py[i-1], py[i]]
    ax.plot(step_x, step_y, color=color, linestyle="--", linewidth=linewidth, zorder=2)

def plot_dataset(ax, pts, color):
    for p in pts:
        pareto = is_pareto(p, pts)
        marker = "D" if pareto else "o"
        size   = 120 if pareto else 55
        ax.scatter(p["mean_ags"], p["stability"],
                   color=color, s=size, marker=marker,
                   zorder=4 if pareto else 3,
                   edgecolors="black" if pareto else "#888888",
                   linewidths=1.5 if pareto else 0.5)
        if pareto:
            ax.annotate(f"k={p['k']},q={p['t']}",
                        (p["mean_ags"], p["stability"]),
                        xytext=(p["mean_ags"] + 0.15, p["stability"] + 0.005),
                        fontsize=8)
    draw_pareto_step(ax, pts, color)

def load_costs(dataset):
    """Return {k: total_cost_usd} from threshold_sweep.json for each level."""
    costs = {}
    for v in range(1, 8):
        f = HERE / "experiments" / dataset / f"v{v}" / "threshold_sweep.json"
        if not f.exists():
            continue
        d = json.loads(f.read_text())
        costs[v] = d.get("costs", {}).get("total", {}).get("cost_usd", 0.0)
    return costs

# ── Figure size tuning ───────────────────────────────────────────────────────
# Set GRID_FIGSIZE to the ACTUAL output size you want in the paper (inches).
# If LaTeX includes it with \includegraphics[width=\textwidth]{...} and your
# \textwidth is 7 in, set this to (7, 5).  Fonts will then appear exactly at
# the sizes specified below — no LaTeX scaling distortion.
GRID_FIGSIZE  = (7.14, 4.8)       # (width_in, height_in) — tune to match LaTeX output
TITLE_FS      = 6           # panel title font size  (pt)
AXLABEL_FS    = 5           # axis label font size   (pt)
TICK_FS       = 4           # tick label font size   (pt)
ANNOT_FS      = 4           # k=x,q=y annotation font size (pt)
LEGEND_FS     = 5           # legend font size       (pt)
PENDING_FS    = 6           # "pending" placeholder text

# ── grid mode: 2x3 figure — six Pareto panels ────────────────────────────────
def plot_grid(k_colors):
    exp_dir = HERE / "experiments"
    datasets_with_data = {
        d.name for d in exp_dir.iterdir()
        if d.is_dir() and any((d / f"v{v}/threshold_sweep.json").exists() for v in range(1, 8))
    }

    fig, axes = plt.subplots(2, 3, figsize=GRID_FIGSIZE)
    axes_flat = axes.flatten()  # indices 0–5

    for slot, ds in enumerate(DATASET_ORDER):
        ax = axes_flat[slot]
        label = DATASET_LABELS.get(ds, ds)

        if ds not in datasets_with_data:
            ax.set_facecolor("#f8f8f8")
            ax.text(0.5, 0.5, f"{label}\n\n(pending)",
                    ha="center", va="center", fontsize=PENDING_FS, color="#999999",
                    transform=ax.transAxes, linespacing=1.6)
            ax.set_title(label.replace("\n", " "), fontsize=TITLE_FS, fontweight="bold", color="#aaaaaa")
            ax.set_xlabel("ASC Retention", fontsize=AXLABEL_FS)
            ax.set_ylabel("Stability", fontsize=AXLABEL_FS)
            ax.grid(True, alpha=0.15)
            ax.tick_params(labelsize=TICK_FS)
            continue

        pts = load_points(ds)
        for p in pts:
            pareto = is_pareto(p, pts)
            ax.scatter(p["mean_ags"], p["stability"],
                       color=k_colors[p["k"]],
                       s=29 if pareto else 15,
                       marker="D" if pareto else "o",
                       zorder=4 if pareto else 3,
                       edgecolors="black" if pareto else "#999999",
                       linewidths=0.8 if pareto else 0.3)
            ax.annotate(f"k={p['k']},q={p['t']}",
                        (p["mean_ags"], p["stability"]),
                        xytext=(p["mean_ags"] + 0.05, p["stability"] + 0.003),
                        fontsize=ANNOT_FS, color="#222222")

        draw_pareto_step(ax, pts, "#333333", linewidth=1.4)
        ax.set_title(label.replace("\n", " "), fontsize=TITLE_FS, fontweight="bold")
        ax.set_xlabel("ASC Retention", fontsize=AXLABEL_FS)
        ax.set_ylabel("Stability (pairwise ASC coverage)", fontsize=AXLABEL_FS)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=TICK_FS)

    # ── Shared k-colour legend (bottom of figure) ─────────────────────────────
    k_handles = [mpatches.Patch(color=k_colors[k], label=f"k={k}")
                 for k in sorted(k_colors)]
    shape_handles = [
        plt.scatter([], [], marker="D", color="white", edgecolors="black",
                    s=29, linewidths=0.8, label="Pareto optimal"),
        plt.scatter([], [], marker="o", color="white", edgecolors="#999999",
                    s=15, linewidths=0.3, label="Non-optimal"),
        plt.Line2D([0], [0], linestyle="--", color="#333333",
                   linewidth=1.0, label="Pareto front"),
    ]
    fig.legend(handles=k_handles + shape_handles,
               fontsize=LEGEND_FS, loc="lower center", ncol=8,
               bbox_to_anchor=(0.5, 0.0), framealpha=0.9)

    plt.tight_layout(rect=[0, 0.06, 1, 1])
    out = exp_dir / "pareto_grid.pdf"
    plt.savefig(out, bbox_inches="tight")
    print(f"Saved: {out}")
    plt.show()

# Slot ordering for --panels: 3 cols × 2 rows
# Row 1: dronboard-early | dronboard | dronology
# Row 2: px4 | aerostack2 | ardupilot
PANELS_ORDER = ["dronboard-early", "dronboard", "dronology", "px4", "aerostack2", "ardupilot"]

# ── panels mode: 3×2 grid (3 cols, 2 rows) — five Pareto panels + cost chart ─
def plot_panels(datasets_with_data, k_colors):
    fig, axes = plt.subplots(2, 3, figsize=(21, 13))
    axes_flat = axes.flatten()  # indices 0–5

    # ── Slots 0–4: one Pareto panel per dataset ──────────────────────────────
    for slot, ds in enumerate(PANELS_ORDER):
        ax = axes_flat[slot]
        label = DATASET_LABELS.get(ds, ds)

        if ds not in datasets_with_data:
            ax.set_facecolor("#f8f8f8")
            ax.text(0.5, 0.5, f"{label}\n\n(pending)",
                    ha="center", va="center", fontsize=13, color="#999999",
                    transform=ax.transAxes, linespacing=1.6)
            ax.set_title(label.replace("\n", " "), fontsize=12, fontweight="bold", color="#aaaaaa")
            ax.set_xlabel("Mean ASCs in model", fontsize=11)
            ax.set_ylabel("Stability", fontsize=11)
            ax.grid(True, alpha=0.15)
            ax.tick_params(labelsize=12)
            continue

        pts = load_points(ds)
        for p in pts:
            pareto = is_pareto(p, pts)
            ax.scatter(p["mean_ags"], p["stability"],
                       color=k_colors[p["k"]],
                       s=110 if pareto else 50,
                       marker="D" if pareto else "o",
                       zorder=4 if pareto else 3,
                       edgecolors="black" if pareto else "#999999",
                       linewidths=1.4 if pareto else 0.5)
            ax.annotate(f"{p['k']}/{p['t']}",
                        (p["mean_ags"], p["stability"]),
                        xytext=(p["mean_ags"] + 0.15, p["stability"] + 0.004),
                        fontsize=11, color="#222222")

        draw_pareto_step(ax, pts, "#333333")
        ax.set_title(label.replace("\n", " "), fontsize=12, fontweight="bold")
        ax.set_xlabel("Mean ASCs in model", fontsize=11)
        ax.set_ylabel("Stability (pairwise ASC coverage)", fontsize=11)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=12)

    # ── Shared k-colour legend (bottom of figure) ─────────────────────────────
    k_handles = [mpatches.Patch(color=k_colors[k], label=f"k={k} (total voters)")
                 for k in sorted(k_colors)]
    shape_handles = [
        plt.scatter([], [], marker="D", color="white", edgecolors="black",
                    s=80, linewidths=1.4, label="Pareto optimal (◆)"),
        plt.scatter([], [], marker="o", color="white", edgecolors="#999999",
                    s=50, linewidths=0.5, label="Non-optimal (●)"),
        plt.Line2D([0], [0], linestyle="--", color="#333333",
                   linewidth=1.5, label="Pareto front"),
    ]
    note = mpatches.Patch(visible=False, label="Label format: k/q  (ensemble size / quorum)")
    fig.legend(handles=k_handles + shape_handles + [note],
               fontsize=12, loc="lower center", ncol=5,
               bbox_to_anchor=(0.5, 0.0), framealpha=0.9)

    fig.suptitle(
        "RQ1: Stability vs Completeness — All Datasets\n"
        "Color = k (ensemble size)  |  Shape = Pareto status  |  Label = k/q (ensemble/quorum)",
        fontsize=15, y=1.01
    )
    plt.tight_layout(rect=[0, 0.05, 1, 1])
    out = HERE / "experiments" / "pareto_front_panels.png"
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved: {out}")
    plt.show()

# ── mode routing ─────────────────────────────────────────────────────────────
if len(sys.argv) > 1 and sys.argv[1] == "--grid":
    k_colors = {1: "#aec6e8", 2: "#4e9fce", 3: "#1a5c8a", 4: "#f4a261", 5: "#e76f51"}
    plot_grid(k_colors)
    sys.exit(0)

# ── single-dataset mode (original behaviour) ────────────────────────────────
if len(sys.argv) > 1 and sys.argv[1] == "--panels":
    exp_dir = HERE / "experiments"
    datasets_with_data = {
        d.name for d in exp_dir.iterdir()
        if d.is_dir() and any((d / f"v{v}/threshold_sweep.json").exists() for v in range(1, 8))
    }
    k_colors = {1: "#aec6e8", 2: "#4e9fce", 3: "#1a5c8a", 4: "#f4a261", 5: "#e76f51"}
    plot_panels(datasets_with_data, k_colors)
    sys.exit(0)

if len(sys.argv) > 1:
    DATASET = sys.argv[1]
    points  = load_points(DATASET)
    k_colors = {1: "#aec6e8", 2: "#4e9fce", 3: "#1a5c8a", 4: "#f4a261", 5: "#e76f51"}

    fig, ax = plt.subplots(figsize=(9, 6))
    for p in points:
        pareto = is_pareto(p, points)
        marker = "D" if pareto else "o"
        size   = 120 if pareto else 60
        ax.scatter(p["mean_ags"], p["stability"],
                   color=k_colors[p["k"]], s=size, marker=marker, zorder=4 if pareto else 3,
                   edgecolors="black" if pareto else "grey",
                   linewidths=1.5 if pareto else 0.5)
        offset = (0.15, 0.005)
        if p["k"] == 4 and p["t"] == 1:
            offset = (0.15, -0.012)
        ax.annotate(f"{p['k']}/{p['t']}",
                    (p["mean_ags"], p["stability"]),
                    xytext=(p["mean_ags"] + offset[0], p["stability"] + offset[1]),
                    fontsize=10)
    draw_pareto_step(ax, points, "black")

    legend_patches = [mpatches.Patch(color=k_colors[k], label=f"k={k} voters") for k in sorted(k_colors)]
    legend_patches.append(plt.Line2D([0], [0], linestyle="--", color="black", label="Pareto front"))
    legend_patches.append(plt.scatter([], [], marker="D", color="grey",
                                      edgecolors="black", s=80, label="Pareto optimal"))
    ax.legend(handles=legend_patches, fontsize=11, loc="lower right")
    ax.set_xlabel("Mean ASCs in model", fontsize=13)
    ax.set_ylabel("Stability (pairwise ASC coverage)", fontsize=13)
    ax.set_title(f"Stability vs Completeness — {DATASET}\n"
                 f"Pareto front across ensemble size k and quorum q", fontsize=13)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out = HERE / "experiments" / DATASET / "pareto_front.png"
    plt.savefig(out, dpi=150)
    print(f"Saved: {out}")
    plt.show()
    sys.exit(0)

# ── combined multi-dataset mode ──────────────────────────────────────────────
exp_dir   = HERE / "experiments"
datasets  = sorted(
    d.name for d in exp_dir.iterdir()
    if d.is_dir() and any((d / f"v{v}/threshold_sweep.json").exists() for v in range(1, 8))
)

fig, ax = plt.subplots(figsize=(11, 7))

legend_patches = []
for idx, ds in enumerate(datasets):
    color  = DATASET_COLORS[idx % len(DATASET_COLORS)]
    pts    = load_points(ds)
    plot_dataset(ax, pts, color)
    legend_patches.append(mpatches.Patch(color=color, label=ds))

# Shape legend entries
legend_patches.append(
    plt.scatter([], [], marker="D", color="white", edgecolors="black", s=90,
                linewidths=1.5, label="Pareto optimal (◆)")
)
legend_patches.append(
    plt.scatter([], [], marker="o", color="white", edgecolors="#888888", s=60,
                linewidths=0.8, label="Non-optimal (●)")
)
legend_patches.append(
    plt.Line2D([0], [0], linestyle="--", color="#555555", linewidth=1.5,
               label="Pareto front (per dataset)")
)

ax.legend(handles=legend_patches, fontsize=11, loc="lower right")
ax.set_xlabel("Mean ASCs in model", fontsize=13)
ax.set_ylabel("Stability (pairwise ASC coverage)", fontsize=13)
ax.set_title("Stability vs Completeness — All Datasets\n"
             "Pareto front across ensemble size k and quorum q", fontsize=13)
ax.grid(True, alpha=0.3)
plt.tight_layout()

out = exp_dir / "pareto_front_combined.png"
plt.savefig(out, dpi=150)
print(f"Saved: {out}")
plt.show()
