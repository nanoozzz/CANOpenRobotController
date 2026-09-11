#!/usr/bin/env python3
"""
Fitts' law 1D reaching task — trial schedule from hard-coded conditions.

Input
-----
CONDITIONS: an explicit list of (A, W, ID) tuples.

The declared ID is the grouping key: conditions sharing an ID value form one ID
level. Nothing is inferred, so there is no clustering tolerance and no rounding
heuristic to tune.

The Shannon ID is still computed from A and W (ID = log2(A/W + 1)) and checked
against the declared value. A mismatch beyond ID_CHECK_TOL raises, which is what
catches a mistyped A, W, or ID before it silently corrupts the design.

Design
------
  * every condition appears REPS times in total, split evenly across N_GROUPS
  * condition balance forces ID balance exactly, provided every ID level holds
    the same number of conditions (checked below)

Output
------
One CSV per group plus a combined file, columns: index, A, W, ID.
"""

import numpy as np
import pandas as pd
from pathlib import Path

# --------------------------------------------------------------- conditions
# (A, W, ID) — ID is the declared grouping label.
# Widths are exact fractions: 4/3 and 8/9, not 1.333 and 0.889. Truncated
# widths shift the computed ID in the 4th decimal and break the match check.
CONDITIONS = [
    (12.0,     2.0,   2.8074),
    ( 8.0,     4/3,   2.8074),
    (15.0,     2.5,   2.8074),

    (18.0,     2.0,   3.3219),
    (12.0,     4/3,   3.3219),
    ( 8.0,     8/9,   3.3219),

    (27.0,     2.0,   3.8580),
    (18.0,     4/3,   3.8580),
    (12.0,     8/9,   3.8580),

    (27.0,     4/3,   4.4094),
    (18.0,     8/9,   4.4094),
    (20.25,    1.0,   4.4094),

    (27.0,     8/9,   4.9715),
    (30.375,   1.0,   4.9715),
    (15.1875,  0.5,   4.9715),
]

# ----------------------------------------------------------------- settings
REPS         = 12      # trials per condition, over the whole session
N_GROUPS     = 4       # groups to split into

ID_DECIMALS  = 4       # rounding for the ID written to the CSV
ID_CHECK_TOL = 0.01    # bits; max allowed |declared ID - computed ID|
ID_OUTPUT    = "given" # "given" -> the declared ID (one value per level)
                       # "calc"  -> the computed ID, rounded to ID_DECIMALS

MAX_RUN         = 1      # max consecutive trials of the same condition
AVOID_ID_REPEAT = False  # also forbid consecutive trials at the same ID
SEED            = 2026

OUT_DIR = Path("schedule")
PREFIX  = "bal_group"


# ------------------------------------------------------------- construction
def build_conditions(conditions, decimals=ID_DECIMALS, tol=ID_CHECK_TOL):
    if not conditions:
        raise ValueError("CONDITIONS is empty")
    for i, c in enumerate(conditions):
        if len(c) != 3:
            raise ValueError(f"condition {i}: expected (A, W, ID), got {c}")

    df = pd.DataFrame(conditions, columns=["A", "W", "ID_given"]).astype(float)

    if (df.A <= 0).any() or (df.W <= 0).any():
        raise ValueError("A and W must all be positive")
    bad = df.A <= df.W / 2
    if bad.any():
        raise ValueError(f"A sits inside the target for condition(s) "
                         f"{df.index[bad].tolist()}")
    if df.duplicated(subset=["A", "W"]).any():
        dup = df[df.duplicated(subset=["A", "W"], keep=False)]
        raise ValueError(f"duplicate (A, W) conditions:\n{dup.to_string()}")

    # --- computed ID, checked against the declared one --------------------
    df["ID_calc"] = np.log2(df.A / df.W + 1.0)
    df["err"] = (df.ID_calc - df.ID_given).abs()
    off = df.err > tol
    if off.any():
        raise ValueError(
            f"declared ID disagrees with log2(A/W+1) by more than {tol} bits:\n"
            + df.loc[off, ["A", "W", "ID_given", "ID_calc", "err"]]
                .round(6).to_string())

    # --- levels come from the DECLARED ID --------------------------------
    df = df.sort_values(["ID_given", "A"]).reset_index(drop=True)
    levels = np.sort(df.ID_given.unique())
    df["ID_level"] = df.ID_given.map({v: i for i, v in enumerate(levels)})
    df["ID_nominal"] = df.ID_given
    df["ID"] = (df.ID_given if ID_OUTPUT == "given"
                else df.ID_calc).round(decimals)
    df.insert(0, "cond", np.arange(len(df)))

    print("CONDITIONS")
    show = df[["cond", "A", "W", "ID_given", "ID_calc", "err", "ID",
               "ID_level"]].copy()
    show["ID_level"] += 1
    print(show.round(6).to_string(index=False))

    sizes = df.groupby("ID_level").size()
    print(f"\n  {len(df)} conditions -> {len(sizes)} ID levels, "
          f"sizes {sizes.tolist()}")
    print(f"  max |declared - computed| = {df.err.max():.6f} bits "
          f"(tolerance {tol})")
    if sizes.nunique() != 1:
        print("  ! ID levels hold different numbers of conditions; balancing "
              "conditions will NOT balance IDs")

    if len(levels) > 1:
        print(f"  smallest gap between ID levels = {np.diff(levels).min():.4f} bits")

    split = df.groupby("ID_level")["ID"].nunique()
    if (split > 1).any():
        print(f"\n  ! WARNING: {int((split > 1).sum())} ID level(s) write more "
              f"than one value to the ID column at {decimals} dp.")
        for lvl, g in df.groupby("ID_level"):
            if g.ID.nunique() > 1:
                print(f"      level {lvl+1}: {sorted(g.ID.unique())}")
        print("    Grouping and balancing use the declared ID, so the schedule "
              "is correct.\n    Set ID_OUTPUT = 'given' for one value per level.")

    return df.drop(columns=["err"])


def check_feasible(cond_df, reps, n_groups, max_run):
    n_cond = len(cond_df)
    total = n_cond * reps
    print(f"\n{'-'*70}\nALLOCATION")
    print(f"  {n_cond} conditions x {reps} reps = {total} trials")

    if reps % n_groups:
        print(f"  ! {reps} reps does not divide into {n_groups} groups")
        return False
    per_group = total // n_groups
    per_cond = reps // n_groups
    print(f"  {n_groups} groups of {per_group} trials")
    print(f"  per group: {per_cond} trials per condition")

    ok = True
    for lvl, g in cond_df.groupby("ID_level"):
        print(f"    ID {g.ID_nominal.iloc[0]:.4f} | {len(g)} condition(s)"
              f" | {len(g)*per_cond} trials per group | {len(g)*reps} total")
    if cond_df.groupby("ID_level").size().nunique() != 1:
        ok = False

    if per_cond > max_run * (per_group - per_cond + 1):
        print(f"  ! MAX_RUN={max_run} unsatisfiable")
        ok = False
    return ok


# --------------------------------------------------------------- sequencing
def shuffle_constrained(labels, keys, max_run, rng, max_iter=50_000):
    """Permute so no value in `keys` repeats more than max_run times in a row."""
    idx = np.arange(len(labels))
    for _ in range(400):
        idx = rng.permutation(idx)
        for _ in range(max_iter):
            k = keys[idx]
            bad = [i for i in range(max_run, len(k))
                   if len(set(k[i - max_run:i + 1])) == 1]
            if not bad:
                return labels[idx]
            i = int(bad[rng.integers(len(bad))])
            j = int(rng.integers(len(idx)))
            if i != j:
                idx[[i, j]] = idx[[j, i]]
    raise RuntimeError("constraints too tight — relax MAX_RUN / AVOID_ID_REPEAT")


def build_schedule(cond_df, reps, n_groups, max_run, avoid_id_repeat, rng):
    per_cond = reps // n_groups
    cond_to_id = cond_df.set_index("cond")["ID_level"].to_dict()
    rows, last_cond, last_id = [], None, None

    for g in range(n_groups):
        pool = np.repeat(cond_df["cond"].to_numpy(), per_cond)
        keys = (np.array([cond_to_id[c] for c in pool])
                if avoid_id_repeat else pool.copy())
        for _ in range(300):
            order = shuffle_constrained(pool, keys, max_run, rng)
            head_ok = (last_cond is None or order[0] != last_cond)
            if avoid_id_repeat:
                head_ok &= (last_id is None or cond_to_id[order[0]] != last_id)
            if head_ok:
                break
        last_cond, last_id = order[-1], cond_to_id[order[-1]]
        for k, c in enumerate(order, start=1):
            rows.append({"group": g + 1, "trial_in_group": k, "cond": int(c)})

    sched = pd.DataFrame(rows)
    sched.insert(0, "index", np.arange(1, len(sched) + 1))
    return sched.merge(cond_df, on="cond", how="left")


# --------------------------------------------------------------- validation
def validate(sched, cond_df, reps, n_groups, max_run):
    print(f"\n{'-'*70}\nVALIDATION")
    ok = True
    n = len(sched)
    per_cond = reps // n_groups
    per_id = per_cond * cond_df.groupby("ID_level").size().iloc[0]

    ct = pd.crosstab(sched.cond, sched.group)
    print(f"  trials per condition, by group (target {per_cond} everywhere): "
          f"min {ct.values.min()}, max {ct.values.max()}  "
          f"{'OK' if (ct.values == per_cond).all() else 'FAIL'}")
    ok &= bool((ct.values == per_cond).all())
    tot = sched.groupby("cond").size()
    print(f"  trials per condition, total (target {reps}): "
          f"{tot.min()}-{tot.max()}  {'OK' if tot.eq(reps).all() else 'FAIL'}")
    ok &= bool(tot.eq(reps).all())

    it = pd.crosstab(sched.ID_nominal, sched.group)
    print(f"\n  trials per ID, by group (target {per_id} everywhere):")
    print(it.to_string())
    hit = bool((it.values == per_id).all())
    print(f"  -> {'OK' if hit else 'FAIL'}")
    ok &= hit

    sizes = sched.groupby("group").size()
    print(f"\n  group sizes: {sorted(sizes)}  "
          f"{'OK' if sizes.nunique() == 1 else 'FAIL'}")
    ok &= sizes.nunique() == 1

    print(f"\n  temporal spread — mean index per condition "
          f"(uniform = {(n+1)/2:.1f}):")
    for lvl, g in sched.groupby("ID_level"):
        parts = " ".join(f"({a:g},{w:.4g}):{m:5.1f}" for (a, w), m
                         in g.groupby(["A", "W"])["index"].mean().items())
        print(f"    ID {g.ID_nominal.iloc[0]:.4f}: {parts}")
    drift = np.corrcoef(sched.ID_calc, sched["index"])[0, 1]
    print(f"\n  corr(ID, trial index) = {drift:+.4f}  "
          f"{'OK' if abs(drift) < 0.05 else 'CHECK'}")

    runs = (sched.cond != sched.cond.shift()).cumsum()
    longest = int(sched.groupby(runs).size().max())
    print(f"  longest run of one condition: {longest} (cap {max_run})  "
          f"{'OK' if longest <= max_run else 'FAIL'}")
    ok &= longest <= max_run

    print(f"\n  overall: {'PASS' if ok else 'FAIL'}")
    return ok


# --------------------------------------------------------------------- main
if __name__ == "__main__":
    rng = np.random.default_rng(SEED)

    cond_df = build_conditions(CONDITIONS)
    if not check_feasible(cond_df, REPS, N_GROUPS, MAX_RUN):
        raise SystemExit(1)

    sched = build_schedule(cond_df, REPS, N_GROUPS, MAX_RUN,
                           AVOID_ID_REPEAT, rng)
    validate(sched, cond_df, REPS, N_GROUPS, MAX_RUN)

    OUT_DIR.mkdir(exist_ok=True)
    out = sched[["index", "A", "W", "ID"]].copy()
    out["A"] = out["A"].round(4)
    out["W"] = out["W"].round(6)
    out["ID"] = out["ID"].round(ID_DECIMALS)

    print(f"\n{'-'*70}\nFILES")
    for g, grp in sched.groupby("group"):
        path = OUT_DIR / f"{PREFIX}_{g}.csv"
        out.loc[grp.index].to_csv(path, index=False)
        print(f"  {path}  {len(grp)} trials  "
              f"index {grp['index'].min()}-{grp['index'].max()}")

    allpath = OUT_DIR / f"{PREFIX}_all.csv"
    out.to_csv(allpath, index=False)
    print(f"  {allpath}  {len(out)} trials (combined, seed {SEED})")

    print(f"\nfirst 10 rows of {PREFIX}_1.csv:")
    print(out.head(10).to_string(index=False))