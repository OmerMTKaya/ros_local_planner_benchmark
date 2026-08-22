#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_results_builder.py

rate-based indicators are recomputed from trial-level rows using a single
mutually exclusive terminal outcome per trial. Collision outcomes are given
priority over local planner failure when both flags are present in the same row.
This keeps SCR + DCR + LPFR + TR + SAR = 100 for every 100-trial combination.
"""

from __future__ import annotations
import sys
from pathlib import Path
import pandas as pd
import numpy as np

if not hasattr(np, "float"):
    np.float = float
if not hasattr(np, "int"):
    np.int = int
if not hasattr(np, "bool"):
    np.bool = bool

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
TEST_ROOT = SCRIPT_DIR.parents[1]
FULL_STUDY_ROOT = TEST_ROOT / "docs_tables_media" / "full_study"
RAW_ROOT = FULL_STUDY_ROOT / "raw_results"
OUT_ROOT = FULL_STUDY_ROOT / "editted_results"

CONFIGS = ["same", "original"]
WORLDS  = ["bosOrt", "duraganOrt", "hareketliOrt", "karmaOrt"]
GENERATE_PARALLEL_COORDINATES = True

PLANNER_NAME_MAP = {
    "dwa": "DWA",
    "teb": "TEB",
    "trajectory": "Trajectory",
    "hybrid": "Hybrid",
    "neo": "Neo",
}

PLANNER_ORDER = ["DWA", "TEB", "Trajectory", "Hybrid", "Neo"]

PANDAS_READ_CSV_KW = dict(sep=None, engine="python")

COLUMNS = [
    "Local Planner",
    "Average Deviation Time (s)",
    "Average Navigation Time (s)",
    "Average Planned Path Length (m)",
    "Average Executed Path Length (m)",
    "Average Speed (m/s)",
    "Static Collisions (%)",
    "Dynamic Collisions (%)",
    "Local Planner Failure (%)",
    "Timeout (%)",
    "Success (%)",
]

RADAR_COLUMNS = [
    "Local Planner",
    "Average Deviation Time (s) / Average Navigation Time (s)",
    "Path-Length Conformity Score (PLCS)",
    "Average Speed (m/s)",
    "Static Collisions (%)",
    "Dynamic Collisions (%)",
    "Local Planner Failure (%)",
    "Timeout (%)",
    "Success (%)",
]

DISPLAY_LABELS = {
    "Average Deviation Time (s) / Average Navigation Time (s)": "Deviation Time /\nNavigation Time",
    "Path-Length Conformity Score (PLCS)": "Path-Length\nConformity Score",
    "Average Speed (m/s)": "Average\nSpeed",
    "Static Collisions (%)": "Static\nCollisions",
    "Dynamic Collisions (%)": "Dynamic\nCollisions",
    "Local Planner Failure (%)": "Local Planner\nFailure",
    "Timeout (%)": "Timeout",
    "Success (%)": "Success",
}

LOW_IS_BETTER = {
    "Average Deviation Time (s) / Average Navigation Time (s)",
    "Static Collisions (%)",
    "Dynamic Collisions (%)",
    "Local Planner Failure (%)",
    "Timeout (%)",
}

AVG_COLS = [
    "Average Deviation Time (s)",
    "Average Navigation Time (s)",
    "Average Planned Path Length (m)",
    "Average Executed Path Length (m)",
    "Average Speed (m/s)",
]

PCT_COLS = [
    "Static Collisions (%)",
    "Dynamic Collisions (%)",
    "Local Planner Failure (%)",
    "Timeout (%)",
    "Success (%)",
]

TRIAL_TO_AVG = {
    "Average Deviation Time (s)": "Deviation Time (s)",
    "Average Navigation Time (s)": "Navigation Time (s)",
    "Average Planned Path Length (m)": "Planned Path Length (m)",
    "Average Executed Path Length (m)": "Executed Path Length (m)",
    "Average Speed (m/s)": "Average Speed (m/s)",
}

# Terminal-outcome priority.
# Rationale: if a collision and local planner failure are both flagged in a trial,
# the collision is retained as the terminal safety-relevant outcome, and the
# same trial is not counted again as a local planner failure.
TERMINATION_PRIORITY = [
    ("Static Collisions", "Static Collisions (%)"),
    ("Dynamic Collisions", "Dynamic Collisions (%)"),
    ("Timeout", "Timeout (%)"),
    ("Local Planner Failure", "Local Planner Failure (%)"),
    ("Success", "Success (%)"),
]


def infer_planner_from_filename(path: Path) -> str:
    head = path.stem.split("_", 1)[0].lower()
    return PLANNER_NAME_MAP.get(head, head.capitalize())


def _yes(series: pd.Series) -> pd.Series:
    return series.astype(str).str.strip().str.lower().eq("yes")


def _trial_rows(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path, **PANDAS_READ_CSV_KW)
    return df[df["Trial"].astype(str).str.strip().str.lower() != "general result"].copy()


def compute_trial_based_row(csv_path: Path) -> tuple[list, dict]:
    data = _trial_rows(csv_path)
    n_total = len(data)

    values = {}
    for out_col, trial_col in TRIAL_TO_AVG.items():
        values[out_col] = pd.to_numeric(
            data[trial_col].astype(str).str.replace(",", ".", regex=False),
            errors="coerce"
        ).mean()

    flags = {trial_col: _yes(data[trial_col]) for trial_col, _ in TERMINATION_PRIORITY}
    counts = {out_col: 0 for _, out_col in TERMINATION_PRIORITY}
    overlap_trials = []
    zero_trials = []

    priority_map = dict(TERMINATION_PRIORITY)

    for idx in data.index:
        true_flags = [trial_col for trial_col, _ in TERMINATION_PRIORITY if flags[trial_col].loc[idx]]

        if not true_flags:
            zero_trials.append(data.loc[idx, "Trial"])
            continue

        if len(true_flags) > 1:
            overlap_trials.append((data.loc[idx, "Trial"], "+".join(true_flags)))

        selected_trial_col = true_flags[0]
        counts[priority_map[selected_trial_col]] += 1

    for out_col in PCT_COLS:
        values[out_col] = 100.0 * counts[out_col] / n_total if n_total else np.nan

    row = [infer_planner_from_filename(csv_path)] + [values[col] for col in COLUMNS[1:]]

    report = {
        "file": csv_path.name,
        "n_total": n_total,
        "overlap_count": len(overlap_trials),
        "zero_count": len(zero_trials),
        "overlap_trials": "; ".join([f"{t}:{desc}" for t, desc in overlap_trials]),
    }

    for _, out_col in TERMINATION_PRIORITY:
        report[out_col.replace(" (%)", " count")] = counts[out_col]

    return row, report


def collect_rows_for_world(config: str, world: str) -> tuple[list[list], list[dict]]:
    csv_dir = RAW_ROOT / config / world
    rows: list[list] = []
    reports: list[dict] = []

    if not csv_dir.exists():
        print(f"[INFO] Folder not found: {csv_dir}")
        return rows, reports

    files = sorted([p for p in csv_dir.glob("*.csv")] + [p for p in csv_dir.glob("*.CSV")])

    for csv_path in files:
        row, report = compute_trial_based_row(csv_path)
        rows.append(row)
        report.update({"config": config, "world": world, "planner": row[0]})
        reports.append(report)
        print(f"[DBG] {config}/{world} <- {csv_path.name} ({row[0]})")

    rows = sorted(rows, key=lambda r: PLANNER_ORDER.index(r[0]) if r[0] in PLANNER_ORDER else 999)
    return rows, reports


def write_world_table(rows: list[list], config: str, world: str) -> Path:
    out_dir = OUT_ROOT / config / world
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{world}Kiyaslama.csv"

    df = pd.DataFrame(rows, columns=COLUMNS)

    for col in AVG_COLS:
        df[col] = pd.to_numeric(df[col], errors="coerce").round(2)

    for col in PCT_COLS:
        df[col] = pd.to_numeric(df[col], errors="coerce").round(0).astype("Int64")

    df.to_csv(out_path, index=False)
    return out_path


def _read_world_kiyaslama(config: str, world: str) -> pd.DataFrame | None:
    path = OUT_ROOT / config / world / f"{world}Kiyaslama.csv"
    if not path.exists():
        return None
    return pd.read_csv(path, **PANDAS_READ_CSV_KW)


def aggregate_config_table_from_kiyaslamas(config: str) -> pd.DataFrame:
    world_dfs = {w: _read_world_kiyaslama(config, w) for w in WORLDS}
    rows = []

    for planner in PLANNER_ORDER:
        avg_means = {}
        pct_means = {}

        for col in AVG_COLS:
            vals = []
            for w in WORLDS:
                dfw = world_dfs.get(w)
                if dfw is None or dfw.empty:
                    continue
                row = dfw.loc[dfw["Local Planner"] == planner]
                if not row.empty:
                    vals.append(pd.to_numeric(row.iloc[0][col], errors="coerce"))
            avg_means[col] = np.nanmean(vals)

        for col in PCT_COLS:
            vals = []
            for w in WORLDS:
                dfw = world_dfs.get(w)
                if dfw is None or dfw.empty:
                    continue
                row = dfw.loc[dfw["Local Planner"] == planner]
                if not row.empty:
                    vals.append(pd.to_numeric(row.iloc[0][col], errors="coerce"))
            pct_means[col] = np.nanmean(vals)

        dev_avg = avg_means["Average Deviation Time (s)"]
        nav_avg = avg_means["Average Navigation Time (s)"]
        planned_avg = avg_means["Average Planned Path Length (m)"]
        exec_avg = avg_means["Average Executed Path Length (m)"]

        dev_nav_ratio = (dev_avg / nav_avg) if pd.notna(nav_avg) and nav_avg != 0 else np.nan
        if pd.notna(planned_avg) and pd.notna(exec_avg) and planned_avg > 0 and exec_avg > 0:
            plcs = min(planned_avg, exec_avg) / max(planned_avg, exec_avg)
        else:
            plcs = np.nan

        rows.append([
            planner,
            dev_nav_ratio,
            plcs,
            avg_means["Average Speed (m/s)"],
            pct_means["Static Collisions (%)"],
            pct_means["Dynamic Collisions (%)"],
            pct_means["Local Planner Failure (%)"],
            pct_means["Timeout (%)"],
            pct_means["Success (%)"],
        ])

    df = pd.DataFrame(rows, columns=RADAR_COLUMNS)

    for col in RADAR_COLUMNS[1:4]:
        df[col] = pd.to_numeric(df[col], errors="coerce").round(2)

    return df


def write_config_radar(df: pd.DataFrame, config: str) -> Path:
    out_dir = OUT_ROOT / config
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{config}_GenelRadar.csv"
    df.to_csv(out_path, index=False)
    return out_path


def write_combined_average(df_same: pd.DataFrame, df_original: pd.DataFrame) -> Path:
    num_cols = [c for c in RADAR_COLUMNS if c != "Local Planner"]
    m = pd.merge(df_same, df_original, on="Local Planner", how="outer", suffixes=("_same", "_original"))
    m["order"] = m["Local Planner"].apply(lambda x: PLANNER_ORDER.index(x) if x in PLANNER_ORDER else 999)
    m = m.sort_values("order").drop(columns=["order"])

    avg_data = {"Local Planner": m["Local Planner"]}

    for col in num_cols:
        a = pd.to_numeric(m.get(f"{col}_same"), errors="coerce")
        b = pd.to_numeric(m.get(f"{col}_original"), errors="coerce")
        avg_data[col] = (a + b) / 2.0

    df_avg = pd.DataFrame(avg_data, columns=RADAR_COLUMNS)

    for col in RADAR_COLUMNS[1:4]:
        df_avg[col] = pd.to_numeric(df_avg[col], errors="coerce").round(2)

    out_dir = OUT_ROOT / "combined"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "AllWorlds_GenelRadar.csv"
    df_avg.to_csv(out_path, index=False)
    return out_path


def _minmax_norm(series: pd.Series) -> pd.Series:
    s = pd.to_numeric(series, errors="coerce")
    mn, mx = np.nanmin(s.values), np.nanmax(s.values)

    if np.isnan(mn) or np.isnan(mx):
        return pd.Series([np.nan] * len(s), index=series.index)

    if mx - mn == 0:
        return pd.Series([0.5] * len(s), index=series.index)

    return (s - mn) / (mx - mn)


def prepare_parallel_coordinates_matrix(df: pd.DataFrame) -> tuple[list[str], list[str], np.ndarray]:
    df = df.copy()
    df["order"] = df["Local Planner"].apply(lambda x: PLANNER_ORDER.index(x) if x in PLANNER_ORDER else 999)
    df = df.sort_values("order").drop(columns=["order"])

    planners = df["Local Planner"].tolist()
    metrics_raw = [c for c in df.columns if c != "Local Planner"]
    metrics = [DISPLAY_LABELS.get(c, c) for c in metrics_raw]

    norm_cols = []
    for col in metrics_raw:
        norm = _minmax_norm(df[col])
        if col in LOW_IS_BETTER:
            norm = 1 - norm
        norm_cols.append(norm.values)

    data = np.array(norm_cols, dtype=float).T
    return planners, metrics, data


def plot_parallel_coordinates(df: pd.DataFrame, out_png: Path):
    planners, metrics, data = prepare_parallel_coordinates_matrix(df)
    if len(planners) == 0 or len(metrics) == 0:
        return

    x = np.arange(len(metrics))
    fig, ax = plt.subplots(figsize=(11.5, 6.8), dpi=300)

    ax.set_xlim(x[0], x[-1])
    ax.set_ylim(0, 1)
    ax.set_yticks([0.0, 0.25, 0.50, 0.75, 1.0])
    ax.set_yticklabels(["0", "0.25", "0.50", "0.75", "1"], fontsize=13)
    ax.grid(axis="y", linewidth=0.8, alpha=0.45)

    for xi in x:
        ax.axvline(x=xi, color="gray", linewidth=1.0, alpha=0.70, zorder=1)

    for i, planner in enumerate(planners):
        vals = np.nan_to_num(data[i, :], nan=0.0)
        ax.plot(
            x, vals, color=f"C{i}", linewidth=3.0, marker="o",
            markersize=7.5, label=planner, zorder=10, clip_on=False
        )

    ax.set_xticks(x)
    ax.set_xticklabels(metrics, fontsize=13, rotation=45, ha="right", rotation_mode="anchor")
    ax.tick_params(axis="x", pad=8)
    ax.tick_params(axis="y", labelsize=13)
    ax.set_ylabel("Normalized Performance Score", fontsize=14)
    ax.set_xlabel("Performance–Reliability Indicators", fontsize=14, labelpad=0)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    ax.legend(
        loc="upper center", bbox_to_anchor=(0.5, 1.18), ncol=len(planners),
        frameon=False, fontsize=12.5, handlelength=1.6, columnspacing=1.4
    )

    out_png.parent.mkdir(parents=True, exist_ok=True)
    plt.subplots_adjust(left=0.09, right=0.985, top=0.80, bottom=0.30)
    #plt.subplots_adjust(left=0.09, right=0.985, top=0.80, bottom=0.24)
    fig.savefig(out_png, bbox_inches="tight", pad_inches=0.25)
    plt.close(fig)


def main():
    global RAW_ROOT, OUT_ROOT

    if len(sys.argv) >= 2:
        RAW_ROOT = Path(sys.argv[1]).resolve()

    if len(sys.argv) >= 3:
        OUT_ROOT = Path(sys.argv[2]).resolve()

    print(f"[INFO] Raw root = {RAW_ROOT}")
    print(f"[INFO] Output root = {OUT_ROOT}")
    produced = []
    reports = []
    cfg_tables = {}

    for config in CONFIGS:
        for world in WORLDS:
            rows, world_reports = collect_rows_for_world(config, world)
            reports.extend(world_reports)
            p = write_world_table(rows, config, world)
            produced.append(p)
            print(f"[OK] Wrote {p} (rows={len(rows)})")

    report_path = OUT_ROOT / "terminal_outcome_correction_report.csv"
    pd.DataFrame(reports).to_csv(report_path, index=False)
    produced.append(report_path)

    for config in CONFIGS:
        df_cfg = aggregate_config_table_from_kiyaslamas(config)
        p_csv = write_config_radar(df_cfg, config)
        cfg_tables[config] = df_cfg
        produced.append(p_csv)

        if GENERATE_PARALLEL_COORDINATES:
            p_png = Path(p_csv).with_name(Path(p_csv).stem.replace("Radar", "") + "Parallel.png")
            plot_parallel_coordinates(df_cfg, p_png)
            produced.append(p_png)

    if "same" in cfg_tables and "original" in cfg_tables:
        p_csv = write_combined_average(cfg_tables["same"], cfg_tables["original"])
        produced.append(p_csv)

        if GENERATE_PARALLEL_COORDINATES:
            p_png = Path(p_csv).with_name("AllWorlds_Genel_Parallel.png")
            plot_parallel_coordinates(pd.read_csv(p_csv), p_png)
            produced.append(p_png)

    print("\n[SUMMARY] Generated files:")
    for p in produced:
        print(" -", p)


if __name__ == "__main__":
    main()
