import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import linregress, t as t_dist


# ============================================================
# USER SETTINGS
# ============================================================

human_csv  = r"logs/M2FittsHuman_P06_B1_20260924-173601_trials.csv"
robot_csv  = r"logs/M2Fitts_20260922_120830_trials.csv"
shared_csv = r"logs/M2FittsRobotHuman_P06_B2_20260924-180826_trials.csv"

# Block 2 run again with the participant NOT taking part in the reach (they only drag the handle back to the
# origin): the robot's share alone, alpha*u_r. Many trials end as misses; only successful ones are analysed.
# Set to None to skip everything related to this file.
solo_csv   = None #r"logs/M2FittsRobotHuman_P0j_B2_20260922-172801_trials.csv"

# Column mapping for each dataset
#   A, W    : used to RECOMPUTE ID = log2(A/W + 1), so the three files are
#             matched on identical keys (rounding the logged ID column is
#             fragile: the human file stores 4.9715, a tie at 3 decimals)
#   ID_LOG  : ID written by the app; used only as a sanity check
#   MT      : movement-time column
#   PHASE   : (column, value) to keep, or None
#   OK      : (column, value) marking a valid trial, or None
#   EXCLUDE_IF_NONZERO : flag columns; trials with a non-zero flag are dropped

HUMAN = dict(
    name="Human (Block 1)",
    A="A_cm", W="W_cm", ID_LOG="ID_bits", MT="MT_move_s",
    PHASE=("phase", "block"),
    OK=("success", 1),
    EXCLUDE_IF_NONZERO=[],
)

ROBOT = dict(
    name="Robot alone",
    A="D_file", W="W_file", ID_LOG="ID_shannon_bits",
    # NOTE: mt_final_entry_s includes the robot's ~7 ms kinematic RT, whereas
    # the human MT_move_s excludes RT. The RT-free analogue is
    # "mt_final_entry_kin_s". Kept as-is here so that MT_r, MT_o and alpha
    # reproduce the values actually used to run Block 2.
    MT="mt_final_entry_s",
    PHASE=None,
    OK=("success", 1),
    EXCLUDE_IF_NONZERO=[],
)

SHARED = dict(
    name="Shared control (Block 2)",
    A="A_cm", W="W_cm", ID_LOG="ID_bits", MT="MT_move_s",
    PHASE=("phase", "block"),          # Block 2 protocol includes a warm-up
    OK=("success", 1),
    EXCLUDE_IF_NONZERO=["reach_abort"],
)

SOLO = dict(
    name="Robot share only (Block 2, participant not reaching)",
    A="A_cm", W="W_cm", ID_LOG="ID_bits", MT="MT_move_s",   # same app and columns as SHARED
    PHASE=("phase", "block"),
    OK=("success", 1),                 # successful trials only
    EXCLUDE_IF_NONZERO=["reach_abort"],
)
SOLO_COLOR = "purple"

ID_DECIMALS = 3
CI_LEVEL = 0.95

# The shared-control MT-ID relation is NOT expected to be a single Fitts line,
# because alpha (i.e. the controller) changes with ID. A straight-line fit is
# therefore shown only as a clearly labelled overlay in Plot 1, and hidden in
# Plot 2 by default. The same holds for the robot-share-only file (Plot 3).
SHOW_SHARED_OLS_IN_PLOT1 = True
SHOW_SHARED_OLS_IN_PLOT2 = False
SHOW_SOLO_OLS_IN_PLOT3 = False

# Upper y-limit for Plot 1 (None = full range). Trials above the limit are
# drawn as triangles on the top edge so that they remain visible.
PLOT1_YMAX = 1.6

# Plot 3 (the four cases): upper y-limit (None = automatic) and horizontal offset
# between the four series at each ID so that their error bars do not overlap (0 = none).
PLOT3_YMAX = None
PLOT3_DX = 0.00


# ============================================================
# HELPERS
# ============================================================

def signed(value, decimals=4):
    """'+ 0.1234' or '- 0.1234' (avoids the '+ -0.1234' display)."""
    return f"{'-' if value < 0 else '+'} {abs(value):.{decimals}f}"


def load_trials(csv_path, cfg):

    raw = pd.read_csv(csv_path)
    name = cfg["name"]

    required = [cfg["A"], cfg["W"], cfg["MT"]]
    if cfg["PHASE"]:
        required.append(cfg["PHASE"][0])
    if cfg["OK"]:
        required.append(cfg["OK"][0])

    missing = [c for c in required if c not in raw.columns]
    if missing:
        raise ValueError(
            f"\nMissing {name} columns: " + ", ".join(missing)
            + "\n\nAvailable columns:\n" + "\n".join(raw.columns)
        )

    df = raw
    n_raw = len(df)

    if cfg["PHASE"]:
        col, val = cfg["PHASE"]
        df = df[df[col].astype(str).str.strip().str.lower() == str(val).lower()]
    n_phase = len(df)

    if cfg["OK"]:
        col, val = cfg["OK"]
        df = df[pd.to_numeric(df[col], errors="coerce") == val]

    for col in cfg["EXCLUDE_IF_NONZERO"]:
        if col in df.columns:
            df = df[pd.to_numeric(df[col], errors="coerce").fillna(0) == 0]

    out = pd.DataFrame({
        "A":  pd.to_numeric(df[cfg["A"]],  errors="coerce"),
        "W":  pd.to_numeric(df[cfg["W"]],  errors="coerce"),
        "MT": pd.to_numeric(df[cfg["MT"]], errors="coerce"),
    })
    if "alpha" in df.columns:
        out["alpha"] = pd.to_numeric(df["alpha"], errors="coerce")

    out = out.dropna(subset=["A", "W", "MT"])

    # Recompute ID from A and W, and check it against the logged ID
    id_exact = np.log2(out["A"] / out["W"] + 1.0)

    if cfg["ID_LOG"] in df.columns and len(out):
        logged = pd.to_numeric(df.loc[out.index, cfg["ID_LOG"]], errors="coerce")
        max_dev = np.nanmax(np.abs(logged.values - id_exact.values))
        if max_dev > 1e-3:
            print(f"WARNING [{name}]: logged ID differs from log2(A/W+1) "
                  f"by up to {max_dev:.4f} bits")

    out["ID"] = id_exact.round(ID_DECIMALS)

    print(f"{name}: {n_raw} rows -> {n_phase} after phase filter "
          f"-> {len(out)} valid trials used")

    return out.reset_index(drop=True)


def attempts_by_id(csv_path, cfg):
    """Trials attempted and trials validated per ID (after the phase filter), for the success rate."""
    df = pd.read_csv(csv_path)
    if cfg["PHASE"]:
        col, val = cfg["PHASE"]
        df = df[df[col].astype(str).str.strip().str.lower() == str(val).lower()]
    ok = pd.Series(True, index=df.index)
    if cfg["OK"]:
        col, val = cfg["OK"]
        ok &= pd.to_numeric(df[col], errors="coerce") == val
    for col in cfg["EXCLUDE_IF_NONZERO"]:
        if col in df.columns:
            ok &= pd.to_numeric(df[col], errors="coerce").fillna(0) == 0
    ok &= pd.to_numeric(df[cfg["MT"]], errors="coerce").notna()
    ids = np.log2(pd.to_numeric(df[cfg["A"]], errors="coerce")
                  / pd.to_numeric(df[cfg["W"]], errors="coerce") + 1.0).round(ID_DECIMALS)
    out = (pd.DataFrame({"ID": ids, "ok": ok.astype(int)}).dropna(subset=["ID"])
           .groupby("ID")["ok"].agg(n_try="count", n_ok="sum").reset_index())
    out["success_rate"] = out["n_ok"] / out["n_try"]
    return out


def summarise_by_id(df):
    """Observed per-ID statistics (pooled over the A x W conditions)."""
    out = (
        df.groupby("ID")["MT"]
        .agg(n="count", mean="mean", sd="std", median="median")
        .reset_index()
    )
    t_crit = t_dist.ppf(0.5 + CI_LEVEL / 2, out["n"] - 1)
    out["ci_half"] = t_crit * out["sd"] / np.sqrt(out["n"])
    return out


def fit_fitts(df):
    """
    OLS of MT on ID fitted to the A x W condition means -- the same
    procedure as betweenBlocksProcessing.py, so MT_h / MT_r / MT_o / alpha
    here are identical to the values used to run Block 2.
    """
    cond = df.groupby(["A", "W"]).agg(ID=("ID", "first"), MT=("MT", "mean")).reset_index()
    res = linregress(cond["ID"], cond["MT"])
    return dict(
        slope=res.slope, intercept=res.intercept,
        r2=res.rvalue ** 2, p=res.pvalue, se=res.stderr,
        n_points=len(cond),
    )


def predict(fit, x):
    return fit["slope"] * np.asarray(x, dtype=float) + fit["intercept"]


def print_fit(name, fit):
    print(f"\n================ {name} (fit to {fit['n_points']} A×W condition means) ================")
    print(f"MT = {fit['slope']:.6f} × ID {signed(fit['intercept'], 6)}")
    print(f"R² = {fit['r2']:.4f}   p = {fit['p']:.3e}   SE(slope) = {fit['se']:.4f}")


# ============================================================
# LOAD DATA
# ============================================================

human  = load_trials(human_csv,  HUMAN)
robot  = load_trials(robot_csv,  ROBOT)
shared = load_trials(shared_csv, SHARED)
solo   = load_trials(solo_csv, SOLO) if solo_csv else None


# ============================================================
# OBSERVED SUMMARIES AND FITS
# ============================================================

h_sum = summarise_by_id(human)
r_sum = summarise_by_id(robot)
s_sum = summarise_by_id(shared)

h_fit = fit_fitts(human)
r_fit = fit_fitts(robot)
s_fit = fit_fitts(shared)

print_fit("Human", h_fit)
print_fit("Robot", r_fit)
print_fit("Shared control (descriptive only)", s_fit)

if solo is not None:
    solo_sum = summarise_by_id(solo)
    solo_try = attempts_by_id(solo_csv, SOLO)
    if solo["ID"].nunique() >= 2 and solo.groupby(["A", "W"]).ngroups >= 3:
        solo_fit = fit_fitts(solo)
        print_fit("Robot share only (descriptive only)", solo_fit)
    else:
        solo_fit = None
        print("\nRobot share only: too few successful conditions for a straight-line fit")


# ============================================================
# MT_o AND ALPHA (same definition as betweenBlocksProcessing.py)
# ============================================================

ids_common = np.intersect1d(h_sum["ID"], r_sum["ID"])     # human ∩ robot
simplest_ID = ids_common.min()
highest_ID = r_sum["ID"].max()                            # hardest robot ID

MT_h_simple = float(predict(h_fit, simplest_ID))
MT_r_hardest = float(predict(r_fit, highest_ID))

table = pd.DataFrame({"ID": ids_common})
table["MT_h_fit"] = predict(h_fit, table["ID"])
table["MT_r_fit"] = predict(r_fit, table["ID"])

if MT_h_simple > MT_r_hardest:
    print("\nMT_o CASE 1: constant at the human simplest-task MT")
    table["MT_o"] = MT_h_simple
elif np.isclose(highest_ID, simplest_ID):
    table["MT_o"] = MT_h_simple
else:
    print("\nMT_o CASE 2: linear between (simplest ID, MT_h) and (hardest robot ID, MT_r)")
    slope_o = (MT_r_hardest - MT_h_simple) / (highest_ID - simplest_ID)
    table["MT_o"] = MT_h_simple + slope_o * (table["ID"] - simplest_ID)

table["alpha_recomputed"] = (
    (table["MT_o"] - table["MT_h_fit"]) / (table["MT_r_fit"] - table["MT_h_fit"])
)
table.loc[np.isclose(table["MT_r_fit"], table["MT_h_fit"]), "alpha_recomputed"] = np.nan
table["alpha_recomputed"] = table["alpha_recomputed"].clip(0.0, 1.0) + 0.0   # +0.0 removes "-0.00"


# ------------------------------------------------------------
# Cross-check against the alpha actually logged in Block 2
# ------------------------------------------------------------

if "alpha" in shared.columns:
    logged_alpha = shared.groupby("ID")["alpha"].agg(["min", "max"])
    if (logged_alpha["max"] - logged_alpha["min"]).max() > 1e-6:
        print("WARNING: alpha is not constant within an ID in the shared-control log")
    table = table.merge(
        logged_alpha["min"].rename("alpha_logged").reset_index(), on="ID", how="left"
    )
    table["alpha_logged"] = table["alpha_logged"] + 0.0
    diff = (table["alpha_logged"] - table["alpha_recomputed"]).abs().max()
    print(f"\nMax |alpha_logged - alpha_recomputed| = {diff:.4f}")
    if diff > 0.01:
        print("WARNING: the MT_o/alpha recomputed here do not match the alpha used online. "
              "Check that the same human/robot files and settings were used.")
    table["alpha"] = table["alpha_logged"]
else:
    table["alpha"] = table["alpha_recomputed"]

# The robot-share-only run must have used the same alpha per ID as the shared run
if solo is not None and "alpha" in solo.columns and len(solo):
    a_solo = solo.groupby("ID")["alpha"].first().rename("alpha_solo").reset_index()
    chk = table[["ID", "alpha"]].merge(a_solo, on="ID", how="inner")
    if len(chk) and (chk["alpha_solo"] - chk["alpha"]).abs().max() > 1e-6:
        print("WARNING: the robot-share-only file used different alpha values from the shared-control file:\n"
              + chk.to_string(index=False, float_format=lambda v: f"{v:.4f}"))


# ------------------------------------------------------------
# Observed means next to the design values
# ------------------------------------------------------------

def add_obs(tab, summ, tag):
    cols = summ.rename(columns={c: f"{c}_{tag}" for c in summ.columns if c != "ID"})
    return tab.merge(cols, on="ID", how="outer")

table = add_obs(table, h_sum, "h")
table = add_obs(table, r_sum, "r")
table = add_obs(table, s_sum, "s")
table["MT_s_OLS"] = predict(s_fit, table["ID"])
table["s_minus_MT_o"] = table["mean_s"] - table["MT_o"]
table["s_minus_h_mean"] = table["mean_s"] - table["mean_h"]

if solo is not None:
    table = add_obs(table, solo_sum, "solo")
    table = table.merge(solo_try.rename(columns={"n_try": "n_try_solo", "n_ok": "n_ok_solo",
                                                 "success_rate": "success_rate_solo"}), on="ID", how="left")
    table["solo_minus_MT_o"] = table["mean_solo"] - table["MT_o"]
    table["s_minus_solo"] = table["mean_s"] - table["mean_solo"]      # what the participant changed

table = table.sort_values("ID").reset_index(drop=True)

print("\n================ Observed means vs design values ================")
print(
    table[["ID", "alpha", "n_h", "mean_h", "n_s", "mean_s", "median_s",
           "MT_h_fit", "MT_r_fit", "MT_o", "MT_s_OLS",
           "s_minus_MT_o", "s_minus_h_mean"]]
    .to_string(index=False, float_format=lambda v: f"{v:.4f}")
)

if solo is not None:
    print("\n================ Robot share only (participant not reaching): successful trials ================")
    print(
        table[["ID", "alpha", "n_try_solo", "n_ok_solo", "success_rate_solo", "mean_solo", "median_solo",
               "MT_r_fit", "MT_o", "mean_s", "s_minus_solo", "solo_minus_MT_o"]]
        .to_string(index=False, float_format=lambda v: f"{v:.4f}")
    )

summary_output = shared_csv.replace(".csv", "_summary_by_ID.csv")
table.to_csv(summary_output, index=False)
print(f"\nSummary table saved to:\n{summary_output}")


# ============================================================
# PLOT 1 -- SHARED CONTROL: OBSERVED DATA vs TARGET MT_o
# ============================================================

fig, ax = plt.subplots(figsize=(10, 6))
rng = np.random.default_rng(0)
jitter = rng.uniform(-0.035, 0.035, len(shared))

ax.scatter(shared["ID"] + jitter, shared["MT"], s=16, alpha=0.35,
           color="0.45", label="Shared-control trials")

if PLOT1_YMAX is not None:
    over = shared["MT"] > PLOT1_YMAX
    if over.any():
        ax.scatter(shared.loc[over, "ID"] + jitter[over.values],
                   np.full(over.sum(), PLOT1_YMAX * 0.985),
                   marker="^", s=30, color="0.2",
                   label=f"Trials > {PLOT1_YMAX} s ({over.sum()}, clipped)")
    ax.set_ylim(0, PLOT1_YMAX)

ax.errorbar(s_sum["ID"], s_sum["mean"], yerr=s_sum["ci_half"],
            fmt="o", ms=7, capsize=4, color="C3", zorder=5,
            label=f"Observed mean ± {int(CI_LEVEL*100)}% CI")
ax.scatter(s_sum["ID"], s_sum["median"], marker="_", s=300,
           linewidths=2.5, color="C3", zorder=5, label="Observed median")

ax.plot(table["ID"], table["MT_o"], "s-.", color="C2", lw=2,
        label="Target MT_o (design)")

if SHOW_SHARED_OLS_IN_PLOT1:
    xs = np.linspace(shared["ID"].min(), shared["ID"].max(), 200)
    ax.plot(xs, predict(s_fit, xs), ":", color="C0", lw=2,
            label=(f"Straight-line fit: $y = {s_fit['slope']:.3f}x "
                   f"{signed(s_fit['intercept'], 3)}$, $R^2$ = {s_fit['r2']:.2f}"))

for _, row in table.iterrows():
    if pd.notna(row["mean_s"]):
        ax.annotate(f"{row['mean_s']:.3f} s\nα = {row['alpha']:.2f}",
                    (row["ID"], row["mean_s"]), xytext=(9, -4),
                    textcoords="offset points", fontsize=8)

ax.set_xlabel("ID (bits)")
ax.set_ylabel("Movement time (s)")
ax.set_title("Shared control (Block 2): observed MT vs target MT_o")
ax.grid(True, alpha=0.3)
ax.legend(fontsize=8, loc="upper left")
fig.tight_layout()

shared_plot_path = shared_csv.replace(".csv", "_MT_vs_ID.png")
fig.savefig(shared_plot_path, dpi=300, bbox_inches="tight")
print(f"\nShared-control plot saved to:\n{shared_plot_path}")


# ============================================================
# PLOT 2 -- HUMAN vs ROBOT vs SHARED: OBSERVED MEANS
# ============================================================
#
# Markers = observed per-ID means ± CI (these are the values to compare).
# Lines   = the human and robot Fitts fits that define MT_h and MT_r,
#           and the design target MT_o.
# ============================================================

fig, ax = plt.subplots(figsize=(10.5, 6.5))
dx = 0.00                                            # horizontal offset

ax.errorbar(h_sum["ID"] - dx, h_sum["mean"], yerr=h_sum["ci_half"],
            fmt="o", ms=6, capsize=3, color="C0", zorder=5,
            label="Human B1: observed mean ± CI")
ax.errorbar(r_sum["ID"], r_sum["mean"], yerr=r_sum["ci_half"],
            fmt="^", ms=6, capsize=3, color="C1", zorder=5,
            label="Robot alone: observed mean ± CI")
ax.errorbar(s_sum["ID"] + dx, s_sum["mean"], yerr=s_sum["ci_half"],
            fmt="D", ms=6, capsize=3, color="C3", zorder=6,
            label="Shared B2: observed mean ± CI")

xs = np.linspace(table["ID"].min(), table["ID"].max(), 200)
ax.plot(xs, predict(h_fit, xs), "-", color="C0", lw=1.5, alpha=0.7,
        label=f"Human fit (MT_h), $R^2$ = {h_fit['r2']:.2f}")
ax.plot(xs, predict(r_fit, xs), "--", color="C1", lw=1.5, alpha=0.7,
        label=f"Robot fit (MT_r), $R^2$ = {r_fit['r2']:.2f}")
ax.plot(table["ID"], table["MT_o"], "s-.", color="C2", lw=1.8, ms=5,
        label="Target MT_o (design)")

if SHOW_SHARED_OLS_IN_PLOT2:
    ax.plot(xs, predict(s_fit, xs), ":", color="C3", lw=1.5,
            label=f"Shared straight-line fit, $R^2$ = {s_fit['r2']:.2f}")

for _, row in table.iterrows():
    if pd.notna(row["mean_s"]):
        ax.annotate(f"S {row['mean_s']:.3f}\nα={row['alpha']:.2f}",
                    (row["ID"] + dx, row["mean_s"]), xytext=(7, -10),
                    textcoords="offset points", fontsize=7, color="C3")
    if pd.notna(row["mean_h"]):
        ax.annotate(f"H {row['mean_h']:.3f}",
                    (row["ID"] - dx, row["mean_h"]), xytext=(-40, 6),
                    textcoords="offset points", fontsize=7, color="C0")

ax.set_xlabel("ID (bits)")
ax.set_ylabel("Movement time (s)")
ax.set_title("Human vs robot vs shared control: observed means")
ax.grid(True, alpha=0.3)
ax.legend(fontsize=8, loc="upper left")
fig.tight_layout()

comparison_plot_path = shared_csv.replace(".csv", "_Human_Robot_Shared_MT_vs_ID.png")
fig.savefig(comparison_plot_path, dpi=300, bbox_inches="tight")
print(f"\nComparison plot saved to:\n{comparison_plot_path}")


# ============================================================
# PLOT 3 -- HUMAN vs ROBOT vs SHARED vs ROBOT SHARE ONLY: OBSERVED MEANS
# ============================================================
#
# Plot 2 plus the Block 2 re-run in which the participant did not take part in the
# reach (dark green): only the robot's share alpha*u_r moved the handle; successful
# trials only. Its labels give the success count per ID, and IDs without any
# successful trial are marked on the x axis.
# ============================================================

if solo is not None:
    fig, ax = plt.subplots(figsize=(13, 6.5))
    d3 = PLOT3_DX

    ax.errorbar(h_sum["ID"] - 1.5 * d3, h_sum["mean"], yerr=h_sum["ci_half"],
                fmt="o", ms=6, capsize=3, color="C0", zorder=5,
                label="Human B1: observed mean ± CI")
    ax.errorbar(r_sum["ID"] - 0.5 * d3, r_sum["mean"], yerr=r_sum["ci_half"],
                fmt="^", ms=6, capsize=3, color="C1", zorder=5,
                label="Robot alone: observed mean ± CI")
    ax.errorbar(s_sum["ID"] + 0.5 * d3, s_sum["mean"], yerr=s_sum["ci_half"],
                fmt="D", ms=6, capsize=3, color="C3", zorder=6,
                label="Shared B2: observed mean ± CI")
    ax.errorbar(solo_sum["ID"] + 1.5 * d3, solo_sum["mean"], yerr=solo_sum["ci_half"],
                fmt="P", ms=8, capsize=3, color=SOLO_COLOR, zorder=7,
                label="Robot share only (B2, participant not reaching):\nobserved mean ± CI, successful trials")

    xs3 = np.linspace(table["ID"].min(), table["ID"].max(), 200)
    ax.plot(xs3, predict(h_fit, xs3), "-", color="C0", lw=1.5, alpha=0.7,
            label=f"Human fit (MT_h), $R^2$ = {h_fit['r2']:.2f}")
    ax.plot(xs3, predict(r_fit, xs3), "--", color="C1", lw=1.5, alpha=0.7,
            label=f"Robot fit (MT_r), $R^2$ = {r_fit['r2']:.2f}")
    ax.plot(table["ID"], table["MT_o"], "s-.", color="C2", lw=1.8, ms=5,
            label="Target MT_o (design)")

    if SHOW_SOLO_OLS_IN_PLOT3 and solo_fit is not None:
        ax.plot(xs3, predict(solo_fit, xs3), ":", color=SOLO_COLOR, lw=1.5,
                label=f"Robot-share straight-line fit, $R^2$ = {solo_fit['r2']:.2f}")

    if PLOT3_YMAX is not None:
        ax.set_ylim(0, PLOT3_YMAX)
    else:
        ax.set_ylim(bottom=0)
    y_low = ax.get_ylim()[0] + 0.03 * (ax.get_ylim()[1] - ax.get_ylim()[0])

    for _, row in table.iterrows():
        if pd.notna(row["mean_s"]):
            ax.annotate(f"S {row['mean_s']:.3f}\nα={row['alpha']:.2f}",
                        (row["ID"] + 0.5 * d3, row["mean_s"]), xytext=(7, -10),
                        textcoords="offset points", fontsize=7, color="C3")
        if pd.notna(row["mean_h"]):
            ax.annotate(f"H {row['mean_h']:.3f}",
                        (row["ID"] - 1.5 * d3, row["mean_h"]), xytext=(-40, 6),
                        textcoords="offset points", fontsize=7, color="C0")
        n_ok, n_try = row.get("n_ok_solo"), row.get("n_try_solo")
        counts = f"{int(n_ok)}/{int(n_try)} ok" if pd.notna(n_try) else "not run"
        if pd.notna(row.get("mean_solo")):
            ax.annotate(f"Solo {row['mean_solo']:.3f}\n{counts}",
                        (row["ID"] + 1.5 * d3, row["mean_solo"]), xytext=(8, 16),
                        textcoords="offset points", fontsize=7, color=SOLO_COLOR)
        elif pd.notna(row["ID"]):                 # no successful robot-share-only trial at this ID
            ax.annotate(f"Solo: no success\n({counts})", (row["ID"], y_low),
                        ha="center", fontsize=7, color=SOLO_COLOR)

    ax.set_xlabel("ID (bits)")
    ax.set_ylabel("Movement time (s)")
    ax.set_title("Human vs robot vs shared control vs robot share only: observed means")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.01, 1.0), borderaxespad=0.)   # outside the axes
    fig.tight_layout()

    four_plot_path = solo_csv.replace(".csv", "_Human_Robot_Shared_Solo_MT_vs_ID.png")
    fig.savefig(four_plot_path, dpi=300, bbox_inches="tight")
    print(f"\nFour-case comparison plot saved to:\n{four_plot_path}")

plt.show()