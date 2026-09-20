import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress


# ============================================================
# USER SETTINGS
# ============================================================

# ------------------------------------------------------------
# Human CSV
# ------------------------------------------------------------

human_csv = r"logs/M2FittsHuman_P01_B1_20260919-152657_trials.csv"

HUMAN_ID_COLUMN = "ID_bits"
HUMAN_MT_COLUMN = "MT_move_s"


# ------------------------------------------------------------
# Robot CSV
# ------------------------------------------------------------

robot_csv = r"logs/M2Fitts_20260911_190239_trials.csv"

ROBOT_ID_COLUMN = "ID_shannon_bits"
ROBOT_MT_COLUMN = "mt_final_entry_s"


# ------------------------------------------------------------
# Shared-control CSV
# ------------------------------------------------------------
#
# Change this to your shared-control file.
#
# The shared-control CSV is expected to contain:
#
#     ID
#     MT
#
# If your columns have different names, change the two
# variables below.
#

shared_csv = r"logs/M2FittsRobotHuman_P01_B2_20260920-174200_trials.csv"

SHARED_ID_COLUMN = "ID_bits"
SHARED_MT_COLUMN = "MT_move_s"


# ------------------------------------------------------------
# ID grouping
# ------------------------------------------------------------

ID_DECIMALS = 3


# ============================================================
# LOAD AND PREPARE DATA
# ============================================================

def load_data(csv_path, id_column, mt_column, name):

    df = pd.read_csv(csv_path)

    print(f"\n{name} CSV columns:")
    print(df.columns.tolist())

    required = [
        id_column,
        mt_column
    ]

    missing = [
        col for col in required
        if col not in df.columns
    ]

    if missing:
        raise ValueError(
            f"\nMissing {name} columns: "
            + ", ".join(missing)
            + "\n\nAvailable columns:\n"
            + "\n".join(df.columns)
        )

    df = df[
        [
            id_column,
            mt_column
        ]
    ].copy()

    df[id_column] = pd.to_numeric(
        df[id_column],
        errors="coerce"
    )

    df[mt_column] = pd.to_numeric(
        df[mt_column],
        errors="coerce"
    )

    df = df.dropna(
        subset=[
            id_column,
            mt_column
        ]
    )

    # Group tiny floating-point differences
    df[id_column] = (
        df[id_column].round(ID_DECIMALS)
    )

    return df


# ============================================================
# LOAD DATA
# ============================================================

human = load_data(
    human_csv,
    HUMAN_ID_COLUMN,
    HUMAN_MT_COLUMN,
    "Human"
)

robot = load_data(
    robot_csv,
    ROBOT_ID_COLUMN,
    ROBOT_MT_COLUMN,
    "Robot"
)

shared = load_data(
    shared_csv,
    SHARED_ID_COLUMN,
    SHARED_MT_COLUMN,
    "Shared control"
)


# ============================================================
# STANDARDISE COLUMN NAMES
# ============================================================
#
# From this point onward everything simply uses:
#
#     ID
#     MT
#

human = human.rename(
    columns={
        HUMAN_ID_COLUMN: "ID",
        HUMAN_MT_COLUMN: "MT"
    }
)

robot = robot.rename(
    columns={
        ROBOT_ID_COLUMN: "ID",
        ROBOT_MT_COLUMN: "MT"
    }
)

shared = shared.rename(
    columns={
        SHARED_ID_COLUMN: "ID",
        SHARED_MT_COLUMN: "MT"
    }
)


# ============================================================
# REGRESSION FUNCTION
# ============================================================

def calculate_regression(df):

    grouped = (
        df.groupby("ID")["MT"]
        .agg(
            [
                "mean",
                "std",
                "count"
            ]
        )
        .reset_index()
    )

    x = grouped["ID"].values
    y = grouped["mean"].values

    if len(x) < 2:
        raise ValueError(
            "Not enough unique ID values for regression."
        )

    result = linregress(x, y)

    slope = result.slope
    intercept = result.intercept

    grouped["MT_expected"] = (
        slope * grouped["ID"]
        + intercept
    )

    return {
        "grouped": grouped,
        "slope": slope,
        "intercept": intercept,
        "r_value": result.rvalue,
        "r_squared": result.rvalue ** 2,
        "p_value": result.pvalue,
        "std_err": result.stderr
    }


# ============================================================
# CALCULATE REGRESSIONS
# ============================================================

human_reg = calculate_regression(human)
robot_reg = calculate_regression(robot)
shared_reg = calculate_regression(shared)


# ============================================================
# PRINT REGRESSION RESULTS
# ============================================================

def print_regression(name, reg):

    print(
        f"\n================ {name} Regression ================"
    )

    print(
        f"MT = "
        f"{reg['slope']:.6f} × ID + "
        f"{reg['intercept']:.6f}"
    )

    print(
        f"R²       = {reg['r_squared']:.6f}"
    )

    print(
        f"R        = {reg['r_value']:.6f}"
    )

    print(
        f"p-value  = {reg['p_value']:.6e}"
    )

    print(
        f"SE       = {reg['std_err']:.6f}"
    )


print_regression(
    "Human",
    human_reg
)

print_regression(
    "Robot",
    robot_reg
)

print_regression(
    "Shared Control",
    shared_reg
)


# ============================================================
# EXPECTED MT FOR EACH ID
# ============================================================

human_expected = (
    human_reg["grouped"]
    [
        [
            "ID",
            "MT_expected"
        ]
    ]
    .rename(
        columns={
            "MT_expected": "MT_h"
        }
    )
)

robot_expected = (
    robot_reg["grouped"]
    [
        [
            "ID",
            "MT_expected"
        ]
    ]
    .rename(
        columns={
            "MT_expected": "MT_r"
        }
    )
)

shared_expected = (
    shared_reg["grouped"]
    [
        [
            "ID",
            "MT_expected"
        ]
    ]
    .rename(
        columns={
            "MT_expected": "MT_shared"
        }
    )
)


# ============================================================
# COMBINE EXPECTED VALUES
# ============================================================

expected = human_expected.merge(
    robot_expected,
    on="ID",
    how="outer"
)

expected = expected.merge(
    shared_expected,
    on="ID",
    how="outer"
)

expected = expected.sort_values(
    "ID"
).reset_index(drop=True)


# ============================================================
# PRINT EXPECTED MT
# ============================================================

print(
    "\n================ Expected MT for Each ID ================"
)

print(
    expected.to_string(
        index=False,
        float_format=lambda x: f"{x:.6f}"
    )
)


# ============================================================
# SAVE EXPECTED MT TABLE
# ============================================================

expected_output = (
    shared_csv
    .replace(
        ".csv",
        "_expected_MT.csv"
    )
)

expected.to_csv(
    expected_output,
    index=False
)

print(
    f"\nExpected MT table saved to:\n"
    f"{expected_output}"
)


# ============================================================
# PLOT 1
# SHARED CONTROL ONLY
# ============================================================

plt.figure(
    figsize=(10, 6)
)


# ------------------------------------------------------------
# Shared individual data points
# ------------------------------------------------------------

plt.scatter(
    shared["ID"],
    shared["MT"],
    alpha=0.45,
    label="Shared-control trials"
)


# ------------------------------------------------------------
# Shared group means
# ------------------------------------------------------------

plt.scatter(
    shared_reg["grouped"]["ID"],
    shared_reg["grouped"]["mean"],
    marker="x",
    s=80,
    linewidths=2,
    label="Shared-control mean"
)


# ------------------------------------------------------------
# Shared regression
# ------------------------------------------------------------

x_shared = np.linspace(
    shared["ID"].min(),
    shared["ID"].max(),
    200
)

y_shared = (
    shared_reg["slope"] * x_shared
    + shared_reg["intercept"]
)

plt.plot(
    x_shared,
    y_shared,
    linewidth=2,
    label="Shared-control regression"
)


# ------------------------------------------------------------
# Expected MT for each ID
# ------------------------------------------------------------

plt.scatter(
    shared_reg["grouped"]["ID"],
    shared_reg["grouped"]["MT_expected"],
    marker="D",
    s=55,
    zorder=5,
    label="Expected MT"
)


# ------------------------------------------------------------
# Annotate expected MT
# ------------------------------------------------------------

for _, row in shared_reg["grouped"].iterrows():

    plt.annotate(
        f"{row['MT_expected']:.2f}s",
        (
            row["ID"],
            row["MT_expected"]
        ),
        xytext=(5, 6),
        textcoords="offset points",
        fontsize=8
    )


# ------------------------------------------------------------
# Equation
# ------------------------------------------------------------

equation_text = (
    f"$y = {shared_reg['slope']:.4f}x "
    f"+ {shared_reg['intercept']:.4f}$\n"
    f"$R^2 = {shared_reg['r_squared']:.4f}$\n"
    f"$r = {shared_reg['r_value']:.4f}$"
)

plt.text(
    0.35,
    0.95,
    equation_text,
    transform=plt.gca().transAxes,
    verticalalignment="top",
    bbox=dict(
        boxstyle="round",
        alpha=0.8
    )
)


plt.xlabel("ID (bits)")
plt.ylabel("Movement time (s)")
plt.title("Shared-Control Movement Time vs ID")

plt.grid(
    True,
    alpha=0.3
)

plt.legend()
plt.tight_layout()


shared_plot_path = (
    shared_csv
    .replace(
        ".csv",
        "_MT_vs_ID.png"
    )
)

plt.savefig(
    shared_plot_path,
    dpi=300,
    bbox_inches="tight"
)

print(
    f"\nShared-control plot saved to:\n"
    f"{shared_plot_path}"
)

plt.show(block=False)


# ============================================================
# PLOT 2
# HUMAN vs ROBOT vs SHARED CONTROL
# ============================================================
#
# IMPORTANT:
#
# Human and Robot:
#     Only their expected/regression values are shown.
#
# Shared:
#     Only the actual shared-control data points are shown.
#
# This keeps the graph from becoming cluttered with every
# human and robot trial.
# ============================================================

plt.figure(
    figsize=(10, 6)
)


# ------------------------------------------------------------
# Human expected MT
# ------------------------------------------------------------

plt.plot(
    human_reg["grouped"]["ID"],
    human_reg["grouped"]["MT_expected"],
    marker="o",
    linewidth=2,
    label="Human expected MT"
)


# ------------------------------------------------------------
# Robot expected MT
# ------------------------------------------------------------

plt.plot(
    robot_reg["grouped"]["ID"],
    robot_reg["grouped"]["MT_expected"],
    marker="^",
    linewidth=2,
    linestyle="--",
    label="Robot expected MT"
)


# ------------------------------------------------------------
# Shared-control actual data
# ------------------------------------------------------------
#
# Only the shared-control points are shown.
#

plt.scatter(
    shared["ID"],
    shared["MT"],
    marker="D",
    s=55,
    alpha=0.7,
    label="Shared-control trials"
)


# ------------------------------------------------------------
# Shared-control expected values
# ------------------------------------------------------------

plt.plot(
    shared_reg["grouped"]["ID"],
    shared_reg["grouped"]["MT_expected"],
    linewidth=2,
    linestyle=":",
    marker="D",
    label="Shared-control expected MT"
)


# ------------------------------------------------------------
# Add expected MT labels
# ------------------------------------------------------------

for _, row in expected.iterrows():

    if pd.notna(row["MT_h"]):

        plt.annotate(
            f"H: {row['MT_h']:.2f}s",
            (
                row["ID"],
                row["MT_h"]
            ),
            xytext=(5, 8),
            textcoords="offset points",
            fontsize=7
        )

    if pd.notna(row["MT_r"]):

        plt.annotate(
            f"R: {row['MT_r']:.2f}s",
            (
                row["ID"],
                row["MT_r"]
            ),
            xytext=(5, -14),
            textcoords="offset points",
            fontsize=7
        )

    if pd.notna(row["MT_shared"]):

        plt.annotate(
            f"S: {row['MT_shared']:.2f}s",
            (
                row["ID"],
                row["MT_shared"]
            ),
            xytext=(5, 8),
            textcoords="offset points",
            fontsize=7
        )


# ------------------------------------------------------------
# Formatting
# ------------------------------------------------------------

plt.xlabel("ID (bits)")
plt.ylabel("Movement time (s)")
plt.title(
    "Human vs Robot vs Shared-Control Movement Time"
)

plt.grid(
    True,
    alpha=0.3
)

plt.legend()

plt.tight_layout()


comparison_plot_path = (
    shared_csv
    .replace(
        ".csv",
        "_Human_Robot_Shared_MT_vs_ID.png"
    )
)

plt.savefig(
    comparison_plot_path,
    dpi=300,
    bbox_inches="tight"
)

print(
    f"\nComparison plot saved to:\n"
    f"{comparison_plot_path}"
)

plt.show()