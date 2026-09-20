import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

# ============================================================
# USER SETTINGS
# ============================================================

#csv_path = r"C:\Users\nguyenbaongu\Downloads\M2FittsHuman_P01_B1_20260917-173753_trials 1.csv"
csv_path = r"logs/M2FittsHuman_P01_B1_20260919-152657_trials.csv";

# Choose what to plot:
#
# "A"  -> MT vs A
# "ID" -> MT vs ID
# "W"  -> MT vs W
#
MODE = "ID"

# Number of decimal places used for ID grouping
ID_DECIMALS = 3


# ============================================================
# CHECK MODE
# ============================================================

if MODE not in ["A", "ID", "W"]:
    raise ValueError(
        "MODE must be either 'A', 'ID', or 'W'."
    )


# ============================================================
# LOAD DATA
# ============================================================

df = pd.read_csv(csv_path)

print("\nCSV columns:")
print(df.columns.tolist())


# ============================================================
# CHECK REQUIRED COLUMNS
# ============================================================

required_columns = [
    "A_cm",
    "W_cm",
    "ID_bits",
    "MT_move_s"
]

missing_columns = [
    col for col in required_columns
    if col not in df.columns
]

if missing_columns:
    raise ValueError(
        "\nMissing columns: "
        + ", ".join(missing_columns)
        + "\n\nAvailable columns are:\n"
        + "\n".join(df.columns)
    )


# ============================================================
# KEEP ONLY THE COLUMNS WE NEED
# ============================================================

df = df[
    [
        "A_cm",
        "W_cm",
        "ID_bits",
        "MT_move_s"
    ]
].copy()


# ============================================================
# CONVERT TO NUMERIC
# ============================================================

df["A_cm"] = pd.to_numeric(
    df["A_cm"],
    errors="coerce"
)

df["W_cm"] = pd.to_numeric(
    df["W_cm"],
    errors="coerce"
)

df["ID_bits"] = pd.to_numeric(
    df["ID_bits"],
    errors="coerce"
)

df["MT_move_s"] = pd.to_numeric(
    df["MT_move_s"],
    errors="coerce"
)


# ============================================================
# REMOVE INVALID ROWS
# ============================================================

df = df.dropna(
    subset=[
        "A_cm",
        "W_cm",
        "ID_bits",
        "MT_move_s"
    ]
)


# ============================================================
# GROUP SIMILAR ID VALUES
# ============================================================

# Example:
#
# 3.3217 -> 3.322
# 3.3218 -> 3.322
# 3.3219 -> 3.322
#
# This prevents tiny floating-point differences from creating
# separate ID groups.

df["ID_bits"] = (
    df["ID_bits"].round(ID_DECIMALS)
)


# ============================================================
# SELECT X-AXIS AND GROUPING
# ============================================================

if MODE == "A":

    # --------------------------------------------------------
    # MT vs A
    # --------------------------------------------------------

    x_column = "A_cm"
    x_label = "A (cm)"
    title = "Movement Time vs A"

    # Mean MT for each A + W
    grouping_columns = [
        "A_cm",
        "W_cm"
    ]


elif MODE == "ID":

    # --------------------------------------------------------
    # MT vs ID
    # --------------------------------------------------------

    x_column = "ID_bits"
    x_label = "ID (bits)"
    title = "Movement Time vs ID"

    # Mean MT for each ID + W
    grouping_columns = [
        "ID_bits",
        "W_cm"
    ]


else:  # MODE == "W"

    # --------------------------------------------------------
    # MT vs W
    # --------------------------------------------------------

    x_column = "W_cm"
    x_label = "W (cm)"
    title = "Movement Time vs W"

    # Mean MT for each W + A
    grouping_columns = [
        "W_cm",
        "A_cm"
    ]


# ============================================================
# GROUP DATA
# ============================================================

grouped = (
    df.groupby(
        grouping_columns
    )["MT_move_s"]
    .agg(
        [
            "mean",
            "std",
            "count"
        ]
    )
    .reset_index()
)


print("\n================ Grouped Data ================")

print(
    grouped.to_string(index=False)
)


# ============================================================
# LINEAR REGRESSION
# ============================================================

x = grouped[x_column].values
y = grouped["mean"].values


if len(x) < 2:
    raise ValueError(
        "Not enough data points for linear regression."
    )


result = linregress(x, y)


slope = result.slope
intercept = result.intercept
r_value = result.rvalue
r_squared = r_value ** 2
p_value = result.pvalue
std_err = result.stderr


# ============================================================
# PRINT REGRESSION STATISTICS
# ============================================================

print(
    "\n================ Regression Statistics ================"
)

print(f"Mode        = {MODE}")
print(f"X variable  = {x_column}")
print(f"Slope       = {slope:.6f}")
print(f"Intercept   = {intercept:.6f}")
print(f"R           = {r_value:.6f}")
print(f"R²          = {r_squared:.6f}")
print(f"p-value     = {p_value:.6e}")
print(f"Std. Error  = {std_err:.6f}")


print("\nRegression equation:")

print(
    f"MT_move_s = "
    f"{slope:.6f} × {x_column} + {intercept:.6f}"
)


# ============================================================
# EXPECTED MOVEMENT TIME
# ============================================================

# Expected MT from the regression:
#
# MT_expected = slope * X + intercept

grouped["MT_expected_s"] = (
    slope * grouped[x_column]
    + intercept
)


# ============================================================
# ID MODE:
# CREATE ONE EXPECTED MT VALUE FOR EACH ID
# ============================================================

if MODE == "ID":

    expected_ID = (
        grouped[
            [
                "ID_bits",
                "MT_expected_s"
            ]
        ]
        .drop_duplicates()
        .sort_values("ID_bits")
        .reset_index(drop=True)
    )

    print(
        "\n================ Expected MT for Each ID ================"
    )

    print(
        expected_ID.to_string(index=False)
    )


# ============================================================
# PLOT
# ============================================================

plt.figure(figsize=(10, 6))


# ============================================================
# DETERMINE WHAT COLOUR REPRESENTS
# ============================================================

if MODE == "A":

    # A mode:
    # Different colours = different W

    color_column = "W_cm"


elif MODE == "ID":

    # ID mode:
    # Different colours = different A

    color_column = "A_cm"


else:

    # W mode:
    # Different colours = different A

    color_column = "A_cm"


# ============================================================
# GET UNIQUE COLOUR GROUPS
# ============================================================

color_groups = sorted(
    df[color_column].unique()
)


# Create one fixed colour for each group
colors = plt.cm.tab10(
    np.linspace(0, 1, len(color_groups))
)

group_colors = dict(
    zip(color_groups, colors)
)


# ============================================================
# PLOT INDIVIDUAL MEASUREMENTS
# ============================================================

for group in color_groups:

    group_data = df[
        df[color_column] == group
    ]

    if MODE == "A":

        label = f"W = {group:g}"

    else:

        label = f"A = {group:g}"

    plt.scatter(
        group_data[x_column],
        group_data["MT_move_s"],
        color=group_colors[group],
        alpha=0.5,
        label=label
    )


# ============================================================
# PLOT GROUP MEANS
# ============================================================

if MODE == "A":

    # --------------------------------------------------------
    # A MODE
    # --------------------------------------------------------
    #
    # Grouping:
    #   A + W
    #
    # Colour:
    #   W

    for width in color_groups:

        width_grouped = grouped[
            grouped["W_cm"] == width
        ]

        plt.scatter(
            width_grouped[x_column],
            width_grouped["mean"],
            color=group_colors[width],
            s=80,
            marker="x",
            linewidths=2
        )


elif MODE == "ID":

    # --------------------------------------------------------
    # ID MODE
    # --------------------------------------------------------
    #
    # Grouping:
    #   ID + W
    #
    # Colour:
    #   A
    #
    # Each mean is coloured according to its A value.

    for _, row in grouped.iterrows():

        id_value = row["ID_bits"]
        width_value = row["W_cm"]

        matching_rows = df[
            (df["ID_bits"] == id_value)
            &
            (df["W_cm"] == width_value)
        ]

        if len(matching_rows) == 0:
            continue

        a_value = matching_rows["A_cm"].iloc[0]

        plt.scatter(
            row[x_column],
            row["mean"],
            color=group_colors[a_value],
            s=80,
            marker="x",
            linewidths=2
        )


else:  # MODE == "W"

    # --------------------------------------------------------
    # W MODE
    # --------------------------------------------------------
    #
    # Grouping:
    #   W + A
    #
    # Colour:
    #   A

    for amplitude in color_groups:

        amplitude_grouped = grouped[
            grouped["A_cm"] == amplitude
        ]

        plt.scatter(
            amplitude_grouped[x_column],
            amplitude_grouped["mean"],
            color=group_colors[amplitude],
            s=80,
            marker="x",
            linewidths=2
        )


# ============================================================
# REGRESSION LINE
# ============================================================

x_line = np.linspace(
    x.min(),
    x.max(),
    200
)

y_line = (
    slope * x_line
    + intercept
)

plt.plot(
    x_line,
    y_line,
    linewidth=2,
    color="black",
    label="Linear regression"
)


# ============================================================
# HIGHLIGHT EXPECTED MT FOR EACH ID
# ============================================================

if MODE == "ID":

    # --------------------------------------------------------
    # Get unique ID -> expected MT pairs
    # --------------------------------------------------------

    expected_ID = (
        grouped[
            [
                "ID_bits",
                "MT_expected_s"
            ]
        ]
        .drop_duplicates()
        .sort_values("ID_bits")
    )

    # Plot expected values as black diamond markers

    plt.scatter(
        expected_ID["ID_bits"],
        expected_ID["MT_expected_s"],
        color="black",
        marker="D",
        s=55,
        zorder=5,
        label="Expected MT"
    )

    # --------------------------------------------------------
    # Add expected MT value next to each ID
    # --------------------------------------------------------

    for _, row in expected_ID.iterrows():

        plt.annotate(
            f"{row['MT_expected_s']:.2f}s",
            (
                row["ID_bits"],
                row["MT_expected_s"]
            ),
            xytext=(5, 6),
            textcoords="offset points",
            fontsize=8
        )


# ============================================================
# EQUATION + STATISTICS
# ============================================================

equation_text = (
    f"$y = {slope:.4f}x + {intercept:.4f}$\n"
    f"$R^2 = {r_squared:.4f}$\n"
    f"$r = {r_value:.4f}$\n"
    f"$p = {p_value:.3e}$\n"
    f"SE = {std_err:.4f}"
)


plt.text(
    0.05,
    0.65,
    equation_text,
    transform=plt.gca().transAxes,
    verticalalignment="top",
    bbox=dict(
        boxstyle="round",
        alpha=0.8
    )
)


# ============================================================
# LABELS / FORMATTING
# ============================================================

plt.xlabel(x_label)
plt.ylabel("Movement time (s)")
plt.title(title)

plt.grid(
    True,
    alpha=0.3
)

plt.legend()

plt.tight_layout()


# ============================================================
# SAVE FIGURE
# ============================================================

if MODE == "A":

    save_path = csv_path.replace(
        ".csv",
        "_MT_move_vs_A.png"
    )

elif MODE == "ID":

    save_path = csv_path.replace(
        ".csv",
        "_MT_move_vs_ID.png"
    )

else:

    save_path = csv_path.replace(
        ".csv",
        "_MT_move_vs_W.png"
    )


plt.savefig(
    save_path,
    dpi=300,
    bbox_inches="tight"
)

print(
    f"\nPlot saved to:\n{save_path}"
)


# ============================================================
# SHOW FIGURE
# ============================================================

plt.show()