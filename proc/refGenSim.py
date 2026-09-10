#this script generates reference signals for the simulation

#!/usr/bin/env python3
"""
Fitts' law 1D reaching task — ID-balanced trial schedule.

Design
------
A x W crossing yields 12 conditions collapsing onto 6 distinct Shannon IDs
(1 / 2 / 3 / 3 / 2 / 1 cells per ID).

Allocation rule
---------------
  * every ID appears exactly TRIALS_PER_ID_PER_ROUND times in every round
  * every round therefore has  n_IDs * TRIALS_PER_ID_PER_ROUND  trials
  * within an ID, its constituent (A, W) cells split that ID's trials as
    evenly as possible, and the split ROTATES across rounds so that each
    cell ends up with exactly the same total over the whole session

Rotation
--------
With k cells sharing an ID and n trials per round, base, rem = divmod(n, k).
Round r gives base+1 trials to cells {r, r+1, ..., r+rem-1} (mod k) and base
to the rest. Over R rounds each cell is bumped R*rem/k times, so all cells
finish with base*R + R*rem/k trials. Exact whenever R*rem is divisible by k.

Output
------
One CSV per round/group with columns: index (1..N global), A, W, ID.
"""

import numpy as np
import pandas as pd
from pathlib import Path

# ----------------------------------------------------------------- settings
A_LEVELS = np.array([8.0, 12.0, 18.0, 27.0])
W_LEVELS = np.array([2.0, 2.0 / 1.5, 2.0 / 1.5 / 1.5])

TRIALS_PER_ID_PER_ROUND = 5
N_ROUNDS                = 6
MAX_RUN                 = 1      # max consecutive trials of the same cell
AVOID_ID_REPEAT         = False  # also forbid consecutive trials at same ID
SEED                    = 2026

OUT_DIR = Path("schedule")
PREFIX  = "group"


# ------------------------------------------------------------- construction
def build_conditions(a_levels, w_levels, decimals=9):
    rows = [
        {"A": a, "W": w, "ratio": a / w, "ID": np.log2(a / w + 1.0)}
        for a in a_levels
        for w in w_levels
    ]
    df = pd.DataFrame(rows).sort_values(["ID", "A"]).reset_index(drop=True)
    key = df["ID"].round(decimals)
    levels = np.sort(key.unique())
    df["ID_level"] = key.map({v: i for i, v in enumerate(levels)})
    df.insert(0, "cell", np.arange(len(df)))
    return df


def check_feasible(cond_df, n_per_round, n_rounds):
    """Report, per ID, whether the rotation closes exactly."""
    print(f"\n{'-'*70}\nALLOCATION")
    print(f"  {n_per_round} trials per ID per round x {n_rounds} rounds "
          f"= {n_per_round*n_rounds} trials per ID")
    print(f"  {cond_df.ID_level.nunique()} IDs -> "
          f"{cond_df.ID_level.nunique()*n_per_round} trials/round, "
          f"{cond_df.ID_level.nunique()*n_per_round*n_rounds} total\n")
    ok = True
    for lvl, grp in cond_df.groupby("ID_level"):
        k = len(grp)
        base, rem = divmod(n_per_round, k)
        bumps = n_rounds * rem
        exact = (bumps % k == 0)
        per_cell = base * n_rounds + bumps // k
        split = f"{base+1}" * 0  # placeholder
        pattern = ("/".join(str(base + 1) for _ in range(rem))
                   + ("/" if rem and rem < k else "")
                   + "/".join(str(base) for _ in range(k - rem)))
        print(f"  ID {grp.ID.iloc[0]:6.4f} | {k} cell(s) | per round {pattern}"
              f" | per cell total {per_cell}"
              f" | {'exact' if exact else 'NOT EXACT — totals will differ by 1'}")
        ok &= exact
    if not ok:
        print("\n  ! rotation does not close; adjust N_ROUNDS or "
              "TRIALS_PER_ID_PER_ROUND")
    return ok


def allocate(cond_df, n_per_round, n_rounds, rng):
    """
    counts[round, cell] — trials of each cell in each round.
    Row block per ID sums to n_per_round; each cell's column sums to its target.
    """
    counts = np.zeros((n_rounds, len(cond_df)), dtype=int)
    for lvl, grp in cond_df.groupby("ID_level"):
        cells = grp["cell"].to_numpy()
        cells = cells[rng.permutation(len(cells))]      # randomise which cell
        k = len(cells)                                  # takes which rotation slot
        base, rem = divmod(n_per_round, k)
        offset = int(rng.integers(k))                   # randomise phase
        for r in range(n_rounds):
            counts[r, cells] += base
            if rem:
                bumped = [(offset + r * rem + i) % k for i in range(rem)]
                counts[r, cells[bumped]] += 1
    return counts


# --------------------------------------------------------------- sequencing
def shuffle_constrained(labels, keys, max_run, rng, max_iter=50_000):
    """
    Permute `labels` so no value in `keys` (aligned with labels) repeats more
    than max_run times in a row. `keys` lets us constrain on cell or on ID.
    """
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
            if i == j:
                continue
            idx[[i, j]] = idx[[j, i]]
    raise RuntimeError("constraints too tight — relax MAX_RUN / AVOID_ID_REPEAT")


def build_schedule(cond_df, counts, max_run, avoid_id_repeat, rng):
    cell_to_id = cond_df.set_index("cell")["ID_level"].to_dict()
    rows, last_cell, last_id = [], None, None

    for r in range(counts.shape[0]):
        pool = np.repeat(np.arange(counts.shape[1]), counts[r])
        keys = (np.array([cell_to_id[c] for c in pool])
                if avoid_id_repeat else pool.copy())
        for _ in range(300):
            order = shuffle_constrained(pool, keys, max_run, rng)
            head_ok = (last_cell is None or order[0] != last_cell)
            if avoid_id_repeat:
                head_ok &= (last_id is None or cell_to_id[order[0]] != last_id)
            if head_ok:
                break
        last_cell, last_id = order[-1], cell_to_id[order[-1]]
        for k, c in enumerate(order, start=1):
            rows.append({"group": r + 1, "trial_in_group": k, "cell": int(c)})

    sched = pd.DataFrame(rows)
    sched.insert(0, "index", np.arange(1, len(sched) + 1))
    return sched.merge(cond_df, on="cell", how="left")


# --------------------------------------------------------------- validation
def validate(sched, cond_df, n_per_round, max_run):
    print(f"\n{'-'*70}\nVALIDATION")
    ok = True

    per = pd.crosstab(sched.ID.round(4), sched.group)
    print("  trials per ID, by group (target "
          f"{n_per_round} everywhere):")
    print(per.to_string())
    hit = bool((per.values == n_per_round).all())
    print(f"  -> {'OK' if hit else 'FAIL'}")
    ok &= hit

    print("\n  trials per (A, W) cell over all 180 trials:")
    tot = sched.groupby(["ID", "A", "W"]).size().rename("n").reset_index()
    for lvl, grp in tot.groupby("ID"):
        vals = grp.n.tolist()
        flag = "equal" if len(set(vals)) == 1 else f"UNEVEN {vals}"
        cells = ", ".join(f"({a:.0f},{w:.3f})={n}"
                          for a, w, n in zip(grp.A, grp.W, grp.n))
        print(f"    ID {lvl:6.4f}: {cells}   [{flag}]")
        ok &= len(set(vals)) == 1

    n = len(sched)
    print(f"\n  temporal spread — mean index per cell (uniform = {(n+1)/2:.1f}):")
    for lvl, grp in sched.groupby("ID"):
        parts = " ".join(
            f"({a:.0f},{w:.3f}):{m:5.1f}"
            for (a, w), m in grp.groupby(["A", "W"])["index"].mean().items())
        print(f"    ID {lvl:6.4f}: {parts}")
    drift = np.corrcoef(sched.ID, sched["index"])[0, 1]
    print(f"\n  corr(ID, trial index) = {drift:+.4f}  "
          f"{'OK' if abs(drift) < 0.05 else 'CHECK'}")

    runs = (sched.cell != sched.cell.shift()).cumsum()
    longest = int(sched.groupby(runs).size().max())
    print(f"  longest run of one cell: {longest} (cap {max_run})  "
          f"{'OK' if longest <= max_run else 'FAIL'}")
    ok &= longest <= max_run

    print(f"\n  overall: {'PASS' if ok else 'FAIL'}")
    return ok


# --------------------------------------------------------------------- main
if __name__ == "__main__":
    rng = np.random.default_rng(SEED)

    cond_df = build_conditions(A_LEVELS, W_LEVELS)
    print("CONDITIONS")
    print(cond_df.assign(ID_level=cond_df.ID_level + 1)
                 .round(4).to_string(index=False))

    if not check_feasible(cond_df, TRIALS_PER_ID_PER_ROUND, N_ROUNDS):
        raise SystemExit(1)

    counts = allocate(cond_df, TRIALS_PER_ID_PER_ROUND, N_ROUNDS, rng)
    sched = build_schedule(cond_df, counts, MAX_RUN, AVOID_ID_REPEAT, rng)
    validate(sched, cond_df, TRIALS_PER_ID_PER_ROUND, MAX_RUN)

    OUT_DIR.mkdir(exist_ok=True)
    out = sched[["index", "A", "W", "ID"]].copy()
    out["A"] = out["A"].round(4)
    out["W"] = out["W"].round(6)
    out["ID"] = out["ID"].round(4)

    print(f"\n{'-'*70}\nFILES")
    for g, grp in sched.groupby("group"):
        path = OUT_DIR / f"{PREFIX}_{g}.csv"
        out.loc[grp.index].to_csv(path, index=False)
        lo, hi = grp["index"].min(), grp["index"].max()
        print(f"  {path}  {len(grp)} trials  index {lo}-{hi}")

    allpath = OUT_DIR / f"{PREFIX}_all.csv"
    out.to_csv(allpath, index=False)
    print(f"  {allpath}  {len(out)} trials (combined, seed {SEED})")

    print(f"\nfirst 10 rows of {PREFIX}_1.csv:")
    print(out.head(10).to_string(index=False))

