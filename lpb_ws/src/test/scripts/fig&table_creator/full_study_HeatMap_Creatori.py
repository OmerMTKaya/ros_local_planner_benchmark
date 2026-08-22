import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

OUTPUT_DPI = 400
OUTPUT_PNG = SCRIPT_DIR / "full_study_heatmap_w_Kaya_andOthers_400dpi.png"
OUTPUT_PDF = SCRIPT_DIR / "full_study_heatmap_w_Kaya_andOthers.pdf"

# -----------------------------
# Data
# -----------------------------
studies = [
    "Pittner18", "Kim18", "Cybulski19", "Naotunna20", "Apurin23",
    "Gurevin23", "Imamoglu23", "Elbouhy25",
    "Urrea26", "Kaya26", "This Study"
]

# Criteria (SC + DC are merged below into one: Collision Analysis).
# DWB is grouped with the DWA family for the two recent ROS 2/Nav2
# benchmarking studies; therefore, the displayed label is "DWA / DWB".
criteria = [
    "DWA / DWB", "TEB", "Trajectory Rollout", "Hybrid", "Neo",
    "Configuration Sets", "Empty Test Env.", "Static Test Env.",
    "Dynamic Test Env.", "Mixed Test Env.", "Local Planner Failure",
    "Collision Analysis",
    "Multi Metric Usage", "Test Route Diversity", "Total Test Number"
]

# Original data columns order:
# [DWA/DWB, TEB, TR, Hybrid, Neo, Config, EE, SE, DE, ME,
#  LPF, SC, DC, MM, R, Total]
data_raw = np.array([
    [1,1,0,0,0,0,1,1,1,0,0,1,1,4,1,9],       # Pittner18
    [1,1,0,0,0,0,1,1,0,0,0,1,0,2,1,13],      # Kim18
    [1,1,0,0,0,1,0,1,0,1,0,1,1,2,3,270],     # Cybulski19
    [1,1,0,0,0,0,1,1,0,0,0,1,0,4,4,240],     # Naotunna20
    [1,1,1,0,0,1,0,1,1,0,1,1,1,3,2,600],     # Apurin23
    [1,1,1,0,0,1,0,1,0,0,0,1,0,2,6,12],      # Gurevin23
    [1,1,0,0,0,0,1,1,1,0,0,1,1,3,0,42],      # Imamoglu23

    # Elbouhy25:
    # DWB, MPPI and RPP; one reported configuration; static and dynamic
    # scenarios in simulation and the real world; six metrics; two routes;
    # 120 tests in total.
    [1,0,0,0,0,1,0,1,1,0,0,1,1,6,2,120],

    # Urrea26:
    # DWB, RPP and MPPI; one reported configuration; three scenes tested in
    # both directions; five metrics; 15 repetitions per
    # scene-controller-direction cell; 270 tests in total.
    [1,0,0,0,0,1,0,1,1,0,0,0,0,5,6,270],

    # Kaya26:
    # Preliminary Study supplied by the authors.
    [1,1,1,1,1,0,1,1,0,0,0,1,0,5,5,100],

    # This Study
    [1,1,1,1,1,2,1,1,1,1,1,1,1,11,10,4000]
], dtype=float)

assert len(studies) == data_raw.shape[0], (
    "studies and data_raw row counts do not match"
)
assert data_raw.shape[1] == 16, (
    "data_raw must contain 16 columns in the documented order"
)

print("Study count:", len(studies))
print("Raw data shape:", data_raw.shape)

# -----------------------------
# Merge SC + DC into a single column (bitmask style):
# SC -> 1, DC -> 2, both -> 3, none -> 0
# -----------------------------
SC_idx = 11
DC_idx = 12

sc = data_raw[:, SC_idx]
dc = data_raw[:, DC_idx]

collision = (
    (sc > 0).astype(int) * 1
    + (dc > 0).astype(int) * 2
)  # 0, 1, 2, or 3

# Build new data matrix with the merged collision column, dropping SC and DC.
# Keep: columns 0..10, collision, columns 13..15
data = np.column_stack([
    data_raw[:, 0:11],
    collision,
    data_raw[:, 13:16]
]).astype(float)

# -----------------------------
# Column-wise normalization:
# - Zeros mean "not addressed/unknown": exclude from normalization,
#   show as blank (NaN), and do not annotate.
# - If a column has no value > 1 (only binary 1s), keep 1s as 1.0.
# - Otherwise normalize within the column using min/max over non-zero values.
# -----------------------------
normalized = data.copy().astype(float)
mask_zero = data == 0
normalized[mask_zero] = np.nan

for j in range(data.shape[1]):
    col = data[:, j]
    nonzero = col[col > 0]

    if nonzero.size == 0:
        continue

    if nonzero.max() <= 1:
        normalized[col == 1, j] = 1.0
        continue

    min_val = nonzero.min()
    max_val = nonzero.max()

    if max_val == min_val:
        normalized[col > 0, j] = 1.0
    else:
        normalized[col > 0, j] = (
            (col[col > 0] - min_val) / (max_val - min_val)
        )

# -----------------------------
# Plot
# -----------------------------
cell_width = 1.0
cell_height = 0.8

fig_width = cell_width * len(criteria)
fig_height = cell_height * len(studies)

plt.figure(figsize=(fig_width, fig_height), dpi=OUTPUT_DPI)

ax = sns.heatmap(
    normalized,
    cmap="RdYlGn",
    vmin=0,
    vmax=1,
    annot=True,
    fmt=".2f",
    linewidths=0,
    cbar=True,
    cbar_kws={"label": "Normalized Evaluation Intensity"},
    xticklabels=criteria,
    yticklabels=studies,
    mask=np.isnan(normalized)
)

ax.set_xticklabels(criteria, rotation=60, ha="right")
ax.set_yticklabels(studies, rotation=0)

plt.xlabel("Evaluation Criteria")
plt.ylabel("Studies (Chronological)")
# plt.title("Criterion-Wise Normalized Evaluation Intensity Across Studies")

plt.tight_layout()
plt.savefig(OUTPUT_PNG, dpi=OUTPUT_DPI, bbox_inches="tight")
plt.savefig(OUTPUT_PDF, bbox_inches="tight")
plt.show()
