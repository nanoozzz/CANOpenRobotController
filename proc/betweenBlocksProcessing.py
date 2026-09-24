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

human_csv = r"logs/M2FittsHuman_P05_B1_20260924-150543_trials.csv"

# Human:
#   A   -> A_cm
#   W   -> W_cm
#   ID  -> ID_bits
#   MT  -> MT_move_s

# ------------------------------------------------------------
# Human trial filtering
# ------------------------------------------------------------
#
# Only use formal Block 1 trials for human data.
# Warmup trials are excluded.
#

HUMAN_BLOCK_COLUMN = "phase"
HUMAN_PHASE = "block"

# ------------------------------------------------------------
# Robot CSV
# ------------------------------------------------------------

robot_csv = r"logs/M2Fitts_20260922_120830_trials.csv"

# Robot:
#   A   -> D_file
#   W   -> W_file
#   ID  -> ID_shannon_bits
#   MT  -> mt_final_entry_s


# ------------------------------------------------------------
# Schedule CSV
# ------------------------------------------------------------
#
# This is the CSV that will receive the alpha column.
#
# It should contain an "ID" column.
#
# Example:
#
# ID,A,W
# 1.585,10,5
# 2.322,20,5
# ...

schedule_csv = r"schedule/bal_group_all.csv"

alpha_output_csv = r"schedule/bal_group_all_alpha.csv"


# ------------------------------------------------------------
# Plot mode
# ------------------------------------------------------------
#
# "A"  -> MT vs A
# "ID" -> MT vs ID
# "W"  -> MT vs W
# "ALL" -> generate all three plots
#

MODE = "ID"


# ------------------------------------------------------------
# ID grouping
# ------------------------------------------------------------
#
# This prevents tiny floating point differences from creating
# separate IDs.
#
# Example:
#   3.3217 -> 3.322
#   3.3218 -> 3.322
#   3.3219 -> 3.322
#

ID_DECIMALS = 3


# ============================================================
# LOAD HUMAN DATA
# ============================================================

human = pd.read_csv(human_csv)

print("\nHuman CSV columns:")
print(human.columns.tolist())


# ------------------------------------------------------------
# Check required columns
# ------------------------------------------------------------

required_human = [
    "A_cm",
    "W_cm",
    "ID_bits",
    "MT_move_s",
    "phase",
    "success"
]

missing_human = [
    col for col in required_human
    if col not in human.columns
]

if missing_human:
    raise ValueError(
        "\nMissing human columns: "
        + ", ".join(missing_human)
        + "\n\nAvailable columns:\n"
        + "\n".join(human.columns)
    )


# ------------------------------------------------------------
# Keep only the columns we need
# ------------------------------------------------------------

human = human[
    [
        "A_cm",
        "W_cm",
        "ID_bits",
        "MT_move_s",
        "phase",
        "success"
    ]
].copy()


# ------------------------------------------------------------
# Convert numeric columns
# ------------------------------------------------------------

for col in [
    "A_cm",
    "W_cm",
    "ID_bits",
    "MT_move_s"
]:
    human[col] = pd.to_numeric(
        human[col],
        errors="coerce"
    )


# ------------------------------------------------------------
# Remove invalid rows
# ------------------------------------------------------------

human = human.dropna(
    subset=[
        "A_cm",
        "W_cm",
        "ID_bits",
        "MT_move_s"
    ]
)


# ============================================================
# FILTER HUMAN DATA
# ============================================================
#
# Exclude warmup trials.
# Only formal "block" trials are used.
# ============================================================

print("\nHuman phase counts BEFORE filtering:")
print(human["phase"].value_counts(dropna=False))


human = human[
    human["phase"].astype(str).str.strip().str.lower()
    == HUMAN_PHASE
].copy()


print("\nHuman phase filtering:")
print(
    f'Using only phase = "{HUMAN_PHASE}" trials.'
)
print(
    f"Number of human trials after filtering: {len(human)}"
)


# ------------------------------------------------------------
# Keep only successful trials
# ------------------------------------------------------------

human = human[
    pd.to_numeric(human["success"], errors="coerce") == 1
].copy()

print(
    f"Number of successful human trials used: {len(human)}"
)


# ------------------------------------------------------------
# Recompute ID from A and W, then round
# ------------------------------------------------------------
#
# Rounding the logged ID is fragile: the human file stores 4.9715,
# which is a tie at 3 decimals (Python round -> 4.971, pandas
# round -> 4.972). A mismatch would silently drop that ID in the
# inner merge below and leave alpha = NaN in the schedule.
#

human_ID_exact = np.log2(
    human["A_cm"] / human["W_cm"] + 1
)

max_dev = (human["ID_bits"] - human_ID_exact).abs().max()

if max_dev > 1e-3:
    print(
        f"WARNING: logged human ID differs from log2(A/W+1) "
        f"by up to {max_dev:.4f} bits"
    )

human["ID_bits"] = (
    human_ID_exact.round(ID_DECIMALS)
)


# ============================================================
# LOAD ROBOT DATA
# ============================================================

robot = pd.read_csv(robot_csv)

print("\nRobot CSV columns:")
print(robot.columns.tolist())


required_robot = [
    "D_file",
    "W_file",
    "ID_shannon_bits",
    "mt_final_entry_s",
    "success"
]

missing_robot = [
    col for col in required_robot
    if col not in robot.columns
]

if missing_robot:
    raise ValueError(
        "\nMissing robot columns: "
        + ", ".join(missing_robot)
        + "\n\nAvailable columns:\n"
        + "\n".join(robot.columns)
    )


robot = robot[
    pd.to_numeric(robot["success"], errors="coerce") == 1
]

# NOTE: mt_final_entry_s includes the robot's ~7 ms kinematic RT,
# whereas the human MT_move_s excludes RT. The RT-free analogue is
# mt_final_entry_kin_s (switching would lower alpha by <= 0.02 for
# this dataset). Left unchanged so alpha stays as used in Block 2.

robot = robot[
    [
        "D_file",
        "W_file",
        "ID_shannon_bits",
        "mt_final_entry_s"
    ]
].copy()


# Convert to numeric
for col in [
    "D_file",
    "W_file",
    "ID_shannon_bits",
    "mt_final_entry_s"
]:
    robot[col] = pd.to_numeric(
        robot[col],
        errors="coerce"
    )


# Remove invalid rows
robot = robot.dropna(
    subset=[
        "D_file",
        "W_file",
        "ID_shannon_bits",
        "mt_final_entry_s"
    ]
)


# Recompute ID from D and W, then round (see note in the human section)
robot_ID_exact = np.log2(
    robot["D_file"] / robot["W_file"] + 1
)

max_dev = (robot["ID_shannon_bits"] - robot_ID_exact).abs().max()

if max_dev > 1e-3:
    print(
        f"WARNING: logged robot ID differs from log2(D/W+1) "
        f"by up to {max_dev:.4f} bits"
    )

robot["ID_shannon_bits"] = (
    robot_ID_exact.round(ID_DECIMALS)
)


# ============================================================
# SIGN-AWARE FORMATTING (avoids "y = 0.2023x + -0.2064")
# ============================================================

def signed(value, decimals=4):
    return f"{'-' if value < 0 else '+'} {abs(value):.{decimals}f}"


# ============================================================
# REGRESSION FUNCTION
# ============================================================

def calculate_regression(
    df,
    x_column,
    mt_column,
    grouping_columns
):
    """
    Calculate grouped mean MT and linear regression.
    """

    grouped = (
        df.groupby(grouping_columns)[mt_column]
        .agg(
            [
                "mean",
                "std",
                "count"
            ]
        )
        .reset_index()
    )

    x = grouped[x_column].values
    y = grouped["mean"].values

    if len(x) < 2:
        raise ValueError(
            f"Not enough data points for regression "
            f"using {x_column}."
        )

    result = linregress(x, y)

    slope = result.slope
    intercept = result.intercept

    r_value = result.rvalue
    r_squared = r_value ** 2
    p_value = result.pvalue
    std_err = result.stderr

    grouped["MT_expected_s"] = (
        slope * grouped[x_column]
        + intercept
    )

    return {
        "grouped": grouped,
        "slope": slope,
        "intercept": intercept,
        "r_value": r_value,
        "r_squared": r_squared,
        "p_value": p_value,
        "std_err": std_err
    }


# ============================================================
# CALCULATE REGRESSIONS
# ============================================================

# ------------------------------------------------------------
# HUMAN REGRESSION
# ------------------------------------------------------------

human_ID_reg = calculate_regression(
    human,
    "ID_bits",
    "MT_move_s",
    [
        "ID_bits",
        "W_cm"
    ]
)


# ------------------------------------------------------------
# ROBOT REGRESSION
# ------------------------------------------------------------

robot_ID_reg = calculate_regression(
    robot,
    "ID_shannon_bits",
    "mt_final_entry_s",
    [
        "ID_shannon_bits",
        "W_file"
    ]
)


# ============================================================
# EXPECTED MT FOR EACH ID
# ============================================================

# ------------------------------------------------------------
# Human expected MT per ID
# ------------------------------------------------------------

human_ID_expected = (
    human_ID_reg["grouped"]
    [
        [
            "ID_bits",
            "MT_expected_s"
        ]
    ]
    .drop_duplicates()
    .sort_values("ID_bits")
    .reset_index(drop=True)
)

human_ID_expected = human_ID_expected.rename(
    columns={
        "ID_bits": "ID",
        "MT_expected_s": "MT_h"
    }
)


# ------------------------------------------------------------
# Robot expected MT per ID
# ------------------------------------------------------------

robot_ID_expected = (
    robot_ID_reg["grouped"]
    [
        [
            "ID_shannon_bits",
            "MT_expected_s"
        ]
    ]
    .drop_duplicates()
    .sort_values("ID_shannon_bits")
    .reset_index(drop=True)
)

robot_ID_expected = robot_ID_expected.rename(
    columns={
        "ID_shannon_bits": "ID",
        "MT_expected_s": "MT_r"
    }
)


# ============================================================
# COMBINE HUMAN + ROBOT EXPECTED MT
# ============================================================

expected_ID = pd.merge(
    human_ID_expected,
    robot_ID_expected,
    on="ID",
    how="inner"
)


# ============================================================
# MT_o
# ============================================================
#
# MT_o represents the movement time of the "operator-only"
# component used in the shared-control alpha calculation.
#
# Two cases are considered:
#
# CASE 1:
#   Human time at the simplest task > robot time at the
#   hardest task.
#
#   In this case, MT_o is constant and equal to the human
#   time for the simplest task.
#
#
# CASE 2:
#   Human time at the simplest task <= robot time at the
#   hardest task.
#
#   In this case, MT_o varies linearly between:
#
#       (simplest ID, human MT)
#
#   and
#
#       (hardest ID, robot MT)
#
# ============================================================


# ------------------------------------------------------------
# Simplest task
# ------------------------------------------------------------

simplest_ID = expected_ID["ID"].min()

MT_h_simple = expected_ID.loc[
    expected_ID["ID"] == simplest_ID,
    "MT_h"
].iloc[0]


# ------------------------------------------------------------
# Hardest robot task
# ------------------------------------------------------------

highest_ID = robot_ID_reg["grouped"]["ID_shannon_bits"].max()

highest_robot = robot_ID_reg["grouped"][
    robot_ID_reg["grouped"]["ID_shannon_bits"] == highest_ID
]

MT_r_hardest = highest_robot["MT_expected_s"].iloc[0]


print("\n=== MT_o endpoints ===")
print(f"Simplest ID       = {simplest_ID:.3f}")
print(f"Human MT simple   = {MT_h_simple:.6f} s")
print(f"Hardest robot ID  = {highest_ID:.3f}")
print(f"Robot MT hardest  = {MT_r_hardest:.6f} s")


# ============================================================
# CALCULATE MT_o
# ============================================================

if MT_h_simple > MT_r_hardest:

    # --------------------------------------------------------
    # CASE 1:
    # Human simplest task is slower than robot hardest task.
    #
    # MT_o is therefore constant.
    # --------------------------------------------------------

    print("\nMT_o CASE 1:")
    print(
        "Human MT at simplest task > "
        "robot MT at hardest task."
    )

    print(
        "MT_o will be constant at the human simplest-task time."
    )

    expected_ID["MT_o"] = MT_h_simple


else:

    # --------------------------------------------------------
    # CASE 2:
    # Human simplest task <= robot hardest task.
    #
    # MT_o follows the straight line connecting:
    #
    #   (simplest_ID, MT_h_simple)
    #
    #   (highest_ID, MT_r_hardest)
    # --------------------------------------------------------

    print("\nMT_o CASE 2:")
    print(
        "Human MT at simplest task <= "
        "robot MT at hardest task."
    )

    print(
        "MT_o will vary linearly between the two endpoints."
    )

    if np.isclose(highest_ID, simplest_ID):

        # Avoid division by zero if there is only one ID.
        expected_ID["MT_o"] = MT_h_simple

    else:

        MT_o_slope = (
            MT_r_hardest - MT_h_simple
        ) / (
            highest_ID - simplest_ID
        )

        MT_o_intercept = (
            MT_h_simple
            - MT_o_slope * simplest_ID
        )

        expected_ID["MT_o"] = (
            MT_o_slope * expected_ID["ID"]
            + MT_o_intercept
        )

        print(
            f"\nMT_o line:"
            f"\nMT_o = {MT_o_slope:.6f} × ID "
            f"{signed(MT_o_intercept, 6)}"
        )


# ============================================================
# PRINT MT_o
# ============================================================

print("\n============================================================")
print("MT_o FOR EACH ID")
print("============================================================")

print(
    expected_ID[
        [
            "ID",
            "MT_h",
            "MT_r",
            "MT_o"
        ]
    ].to_string(
        index=False,
        float_format=lambda x: f"{x:.6f}"
    )
)


# ============================================================
# CALCULATE ALPHA
# ============================================================
#
# alpha =
#
#     (MT_o - MT_h)
#     ----------------
#     (MT_r - MT_h)
#
# ============================================================

expected_ID["alpha"] = (
    expected_ID["MT_o"]
    - expected_ID["MT_h"]
) / (
    expected_ID["MT_r"]
    - expected_ID["MT_h"]
)


# ============================================================
# HANDLE DIVISION BY ZERO
# ============================================================

expected_ID.loc[
    np.isclose(
        expected_ID["MT_r"],
        expected_ID["MT_h"]
    ),
    "alpha"
] = np.nan


# ============================================================
# CLAMP ALPHA TO [0, 1]
# ============================================================

expected_ID["alpha"] = expected_ID["alpha"].clip(
    lower=0.0,
    upper=1.0
) + 0.0   # "+ 0.0" turns -0.0 into 0.0 (the "α: -0.00" label)


# ============================================================
# PRINT EXPECTED MT + MT_o + ALPHA
# ============================================================

print("\n============================================================")
print("EXPECTED MOVEMENT TIME + MT_o + ALPHA FOR EACH ID")
print("============================================================")

print(
    expected_ID.to_string(
        index=False,
        float_format=lambda x: f"{x:.6f}"
    )
)

# ============================================================
# SAVE ID -> MT_h -> MT_r -> ALPHA TABLE
# ============================================================

alpha_table_output = (
    alpha_output_csv
    .replace(".csv", "_ID_alpha_table.csv")
)

expected_ID.to_csv(
    alpha_table_output,
    index=False
)

print(
    f"\nSaved ID/MT/alpha table to:\n"
    f"{alpha_table_output}"
)


# ============================================================
# ADD ALPHA TO SCHEDULE CSV
# ============================================================

schedule = pd.read_csv(schedule_csv)

print("\nSchedule CSV columns:")
print(schedule.columns.tolist())


if "ID" not in schedule.columns:
    raise ValueError(
        "\nThe schedule CSV must contain an 'ID' column."
    )


# Convert schedule ID to numeric
schedule["ID"] = pd.to_numeric(
    schedule["ID"],
    errors="coerce"
)


# Round schedule IDs using the same precision.
# If the schedule has A and W columns, recompute the ID from them
# (same tie-safe key as the human and robot data).
if {"A", "W"}.issubset(schedule.columns):
    schedule["ID"] = np.log2(
        pd.to_numeric(schedule["A"], errors="coerce")
        / pd.to_numeric(schedule["W"], errors="coerce")
        + 1
    ).round(ID_DECIMALS)
else:
    schedule["ID"] = (
        schedule["ID"].round(ID_DECIMALS)
    )


# Create ID -> alpha lookup
alpha_map = dict(
    zip(
        expected_ID["ID"],
        expected_ID["alpha"]
    )
)


# Add alpha based on ID
schedule["alpha"] = (
    schedule["ID"].map(alpha_map)
)

unmatched = schedule["alpha"].isna()

if unmatched.any():
    raise ValueError(
        f"\n{unmatched.sum()} schedule rows received no alpha "
        f"(ID not found in the human/robot table): "
        f"{sorted(schedule.loc[unmatched, 'ID'].unique())}"
    )


# Save
schedule.to_csv(
    alpha_output_csv,
    index=False
)


print(
    f"\nSaved schedule with alpha column to:\n"
    f"{alpha_output_csv}"
)


# ============================================================
# FUNCTION TO CREATE PLOTS
# ============================================================

def make_plot(mode):

    # ========================================================
    # SELECT AXIS
    # ========================================================

    if mode == "A":

        human_x = "A_cm"
        robot_x = "D_file"

        human_xlabel = "A (cm)"
        robot_xlabel = "A (cm)"

        title = "Human vs Robot Movement Time vs A"

        human_grouping = [
            "A_cm",
            "W_cm"
        ]

        robot_grouping = [
            "D_file",
            "W_file"
        ]

        human_color_column = "W_cm"
        robot_color_column = "W_file"

        human_color_label = "W"
        robot_color_label = "W"

        filename = "_Human_vs_Robot_MT_vs_A.png"


    elif mode == "ID":

        human_x = "ID_bits"
        robot_x = "ID_shannon_bits"

        human_xlabel = "ID (bits)"
        robot_xlabel = "ID (bits)"

        title = "Human vs Robot Movement Time vs ID"

        human_grouping = [
            "ID_bits",
            "W_cm"
        ]

        robot_grouping = [
            "ID_shannon_bits",
            "W_file"
        ]

        human_color_column = "A_cm"
        robot_color_column = "D_file"

        human_color_label = "A"
        robot_color_label = "A"

        filename = "_Human_vs_Robot_MT_vs_ID.png"


    elif mode == "W":

        human_x = "W_cm"
        robot_x = "W_file"

        human_xlabel = "W (cm)"
        robot_xlabel = "W (cm)"

        title = "Human vs Robot Movement Time vs W"

        human_grouping = [
            "W_cm",
            "A_cm"
        ]

        robot_grouping = [
            "W_file",
            "D_file"
        ]

        human_color_column = "A_cm"
        robot_color_column = "D_file"

        human_color_label = "A"
        robot_color_label = "A"

        filename = "_Human_vs_Robot_MT_vs_W.png"

    else:
        raise ValueError(
            "Mode must be A, ID, or W."
        )


    # ========================================================
    # REGRESSION
    # ========================================================

    human_reg = calculate_regression(
        human,
        human_x,
        "MT_move_s",
        human_grouping
    )

    robot_reg = calculate_regression(
        robot,
        robot_x,
        "mt_final_entry_s",
        robot_grouping
    )


    # ========================================================
    # PRINT REGRESSION INFORMATION
    # ========================================================

    print("\n============================================================")
    print(f"{mode} MODE REGRESSION")
    print("============================================================")

    print("\nHUMAN")
    print(
        f"MT_h = "
        f"{human_reg['slope']:.6f} × x "
        f"{signed(human_reg['intercept'], 6)}"
    )

    print(
        f"R² = {human_reg['r_squared']:.6f}"
    )

    print("\nROBOT")
    print(
        f"MT_r = "
        f"{robot_reg['slope']:.6f} × x "
        f"{signed(robot_reg['intercept'], 6)}"
    )

    print(
        f"R² = {robot_reg['r_squared']:.6f}"
    )


    # ========================================================
    # CREATE FIGURE
    # ========================================================

    plt.figure(figsize=(11, 7))


    # ========================================================
    # COLOUR GROUPS
    # ========================================================

    all_color_groups = sorted(
        set(
            human[human_color_column].unique()
        ).union(
            set(
                robot[robot_color_column].unique()
            )
        )
    )

    colors = plt.cm.tab10(
        np.linspace(
            0,
            1,
            len(all_color_groups)
        )
    )

    group_colors = dict(
        zip(
            all_color_groups,
            colors
        )
    )


    # ========================================================
    # HUMAN INDIVIDUAL DATA
    # ========================================================

    for group in sorted(
        human[human_color_column].unique()
    ):

        data = human[
            human[human_color_column] == group
        ]

        plt.scatter(
            data[human_x],
            data["MT_move_s"],
            color=group_colors[group],
            alpha=0.30,
            marker="o",
            label=f"Human {human_color_label} = {group:g}"
        )


    # ========================================================
    # ROBOT INDIVIDUAL DATA
    # ========================================================

    for group in sorted(
        robot[robot_color_column].unique()
    ):

        data = robot[
            robot[robot_color_column] == group
        ]

        plt.scatter(
            data[robot_x],
            data["mt_final_entry_s"],
            color=group_colors[group],
            alpha=0.30,
            marker="^",
            label=f"Robot {robot_color_label} = {group:g}"
        )


    # ========================================================
    # HUMAN GROUP MEANS
    # ========================================================

    human_grouped = human_reg["grouped"]

    for _, row in human_grouped.iterrows():

        if mode == "A":
            group = row["W_cm"]

        elif mode == "ID":

            matching = human[
                (human["ID_bits"] == row["ID_bits"])
                &
                (human["W_cm"] == row["W_cm"])
            ]

            if len(matching) == 0:
                continue

            group = matching["A_cm"].iloc[0]

        else:
            group = row["A_cm"]

        plt.scatter(
            row[human_x],
            row["mean"],
            color=group_colors[group],
            marker="o",
            s=90,
            edgecolors="black",
            linewidths=1.2,
            zorder=5
        )


    # ========================================================
    # ROBOT GROUP MEANS
    # ========================================================

    robot_grouped = robot_reg["grouped"]

    for _, row in robot_grouped.iterrows():

        if mode == "A":
            group = row["W_file"]

        elif mode == "ID":

            matching = robot[
                (robot["ID_shannon_bits"] == row["ID_shannon_bits"])
                &
                (robot["W_file"] == row["W_file"])
            ]

            if len(matching) == 0:
                continue

            group = matching["D_file"].iloc[0]

        else:
            group = row["D_file"]

        plt.scatter(
            row[robot_x],
            row["mean"],
            color=group_colors[group],
            marker="^",
            s=100,
            edgecolors="black",
            linewidths=1.2,
            zorder=5
        )


    # Legend entries for the large markers: they are A×W CONDITION
    # means (3 per ID), not per-ID means.
    plt.scatter(
        [], [], marker="o", s=90, facecolors="white",
        edgecolors="black", label="Human A×W condition mean"
    )
    plt.scatter(
        [], [], marker="^", s=100, facecolors="white",
        edgecolors="black", label="Robot A×W condition mean"
    )


    # ========================================================
    # HUMAN REGRESSION LINE
    # ========================================================

    x_min_h = human[human_x].min()
    x_max_h = human[human_x].max()

    x_line_h = np.linspace(
        x_min_h,
        x_max_h,
        200
    )

    y_line_h = (
        human_reg["slope"] * x_line_h
        + human_reg["intercept"]
    )

    plt.plot(
        x_line_h,
        y_line_h,
        linewidth=2,
        linestyle="-",
        label="Human regression"
    )


    # ========================================================
    # ROBOT REGRESSION LINE
    # ========================================================

    x_min_r = robot[robot_x].min()
    x_max_r = robot[robot_x].max()

    x_line_r = np.linspace(
        x_min_r,
        x_max_r,
        200
    )

    y_line_r = (
        robot_reg["slope"] * x_line_r
        + robot_reg["intercept"]
    )

    plt.plot(
        x_line_r,
        y_line_r,
        linewidth=2,
        linestyle="--",
        label="Robot regression"
    )


    # ========================================================
    # ID MODE:
    # SHOW EXPECTED HUMAN / ROBOT MT
    # ========================================================

    if mode == "ID":

        # ----------------------------------------------------
        # Human expected MT
        # ----------------------------------------------------

        human_expected = (
            human_ID_expected
        )

        plt.scatter(
            human_expected["ID"],
            human_expected["MT_h"],
            marker="D",
            s=55,
            zorder=7,
            label="Human expected MT"
        )


        # ----------------------------------------------------
        # Robot expected MT
        # ----------------------------------------------------

        robot_expected = (
            robot_ID_expected
        )

        plt.scatter(
            robot_expected["ID"],
            robot_expected["MT_r"],
            marker="D",
            s=55,
            zorder=7,
            label="Robot expected MT"
        )


        # ----------------------------------------------------
        # Annotate expected values
        # ----------------------------------------------------

        for _, row in expected_ID.iterrows():

            plt.annotate(
                f"H: {row['MT_h']:.2f}s\n"
                f"R: {row['MT_r']:.2f}s\n"
                f"α: {row['alpha']:.2f}",
                (
                    row["ID"],
                    row["MT_h"]
                ),
                xytext=(5, 8),
                textcoords="offset points",
                fontsize=7
            )


    # ========================================================
    # REGRESSION EQUATIONS
    # ========================================================

    equation_text = (
        "Human:\n"
        f"$y = {human_reg['slope']:.4f}x "
        f"{signed(human_reg['intercept'])}$\n"
        f"$R^2 = {human_reg['r_squared']:.4f}$\n\n"

        "Robot:\n"
        f"$y = {robot_reg['slope']:.4f}x "
        f"{signed(robot_reg['intercept'])}$\n"
        f"$R^2 = {robot_reg['r_squared']:.4f}$\n"
        "(fits to A×W condition means)"
    )


    plt.text(
        0.25,
        0.97,
        equation_text,
        transform=plt.gca().transAxes,
        verticalalignment="top",
        bbox=dict(
            boxstyle="round",
            alpha=0.8
        )
    )


    # ========================================================
    # FORMATTING
    # ========================================================

    if mode == "A":
        plt.xlabel("A (cm)")

    elif mode == "ID":
        plt.xlabel("ID (bits)")

    else:
        plt.xlabel("W")


    plt.ylabel("Movement / Entry time (s)")

    plt.title(title)

    plt.grid(
        True,
        alpha=0.3
    )

    plt.legend(
        fontsize=8,
        loc="best"
    )

    plt.tight_layout()


    # ========================================================
    # SAVE
    # ========================================================

    save_path = (
        human_csv.replace(
            ".csv",
            filename
        )
    )

    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )

    print(
        f"\n{mode} plot saved to:\n"
        f"{save_path}"
    )

    plt.show()


# ============================================================
# GENERATE PLOTS
# ============================================================

if MODE == "ALL":

    make_plot("A")
    make_plot("ID")
    make_plot("W")

else:

    make_plot(MODE)


# ============================================================
# FINAL ALPHA SUMMARY
# ============================================================

print("\n============================================================")
print("FINAL ALPHA VALUES")
print("============================================================")

print(
    expected_ID[
        [
            "ID",
            "MT_h",
            "MT_r",
            "alpha"
        ]
    ].to_string(
        index=False,
        float_format=lambda x: f"{x:.6f}"
    )
)

print("\nDone.")