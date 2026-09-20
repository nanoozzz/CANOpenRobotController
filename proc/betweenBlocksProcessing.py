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

# Human:
#   A   -> A_cm
#   W   -> W_cm
#   ID  -> ID_bits
#   MT  -> MT_move_s


# ------------------------------------------------------------
# Robot CSV
# ------------------------------------------------------------

robot_csv = r"logs/M2Fitts_20260911_190239_trials.csv"

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


required_human = [
    "A_cm",
    "W_cm",
    "ID_bits",
    "MT_move_s"
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


human = human[
    [
        "A_cm",
        "W_cm",
        "ID_bits",
        "MT_move_s"
    ]
].copy()


# Convert to numeric
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


# Remove invalid rows
human = human.dropna(
    subset=[
        "A_cm",
        "W_cm",
        "ID_bits",
        "MT_move_s"
    ]
)


# Round ID
human["ID_bits"] = (
    human["ID_bits"].round(ID_DECIMALS)
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
    "mt_final_entry_s"
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


# Round ID
robot["ID_shannon_bits"] = (
    robot["ID_shannon_bits"].round(ID_DECIMALS)
)


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
# MT_o = human movement time for the simplest task.
#
# Here, the simplest task is defined as the smallest ID
# present in BOTH datasets.
#


if len(expected_ID) == 0:
    raise ValueError(
        "No matching IDs were found between human and robot data."
    )


simplest_ID = expected_ID["ID"].min()


MT_o = expected_ID.loc[
    expected_ID["ID"] == simplest_ID,
    "MT_h"
].iloc[0]


print("\n============================================================")
print("SIMPLEST TASK")
print("============================================================")

print(f"Simplest ID = {simplest_ID:.3f}")
print(f"MT_o        = {MT_o:.6f} s")


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
#


expected_ID["alpha"] = (
    (MT_o - expected_ID["MT_h"])
    /
    (
        expected_ID["MT_r"]
        - expected_ID["MT_h"]
    )
)


# Handle division by zero
expected_ID.loc[
    np.isclose(
        expected_ID["MT_r"],
        expected_ID["MT_h"]
    ),
    "alpha"
] = np.nan


# ============================================================
# PRINT EXPECTED MT + ALPHA
# ============================================================

print("\n============================================================")
print("EXPECTED MOVEMENT TIME + ALPHA FOR EACH ID")
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


# Round schedule IDs using the same precision
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
        robot_xlabel = "W"

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
        f"{human_reg['slope']:.6f} × x + "
        f"{human_reg['intercept']:.6f}"
    )

    print(
        f"R² = {human_reg['r_squared']:.6f}"
    )

    print("\nROBOT")
    print(
        f"MT_r = "
        f"{robot_reg['slope']:.6f} × x + "
        f"{robot_reg['intercept']:.6f}"
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
        f"+ {human_reg['intercept']:.4f}$\n"
        f"$R^2 = {human_reg['r_squared']:.4f}$\n\n"

        "Robot:\n"
        f"$y = {robot_reg['slope']:.4f}x "
        f"+ {robot_reg['intercept']:.4f}$\n"
        f"$R^2 = {robot_reg['r_squared']:.4f}$"
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