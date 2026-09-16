import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

# ============================================================
# USER SETTINGS
# ============================================================

csv_path = r"logs/M2Fitts_20260911_190239_trials.csv"

# Choose what to plot:
#
# "A"  -> MT vs A
# "ID" -> MT vs ID
# "W"  -> MT vs W
#
MODE = "W"

# Width column in your CSV
WIDTH_COLUMN = "W_file"

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
    "D_file",
    WIDTH_COLUMN,
    "ID_shannon_bits",
    "mt_final_entry_s"
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
        + f"\n\nChange WIDTH_COLUMN = '{WIDTH_COLUMN}' "
          "to the correct width column name."
    )


# ============================================================
# KEEP ONLY THE COLUMNS WE NEED
# ============================================================

df = df[
    [
        "D_file",
        WIDTH_COLUMN,
        "ID_shannon_bits",
        "mt_final_entry_s"
    ]
].copy()


# ============================================================
# CONVERT TO NUMERIC
# ============================================================

df["D_file"] = pd.to_numeric(
    df["D_file"],
    errors="coerce"
)

df[WIDTH_COLUMN] = pd.to_numeric(
    df[WIDTH_COLUMN],
    errors="coerce"
)

df["ID_shannon_bits"] = pd.to_numeric(
    df["ID_shannon_bits"],
    errors="coerce"
)

df["mt_final_entry_s"] = pd.to_numeric(
    df["mt_final_entry_s"],
    errors="coerce"
)


# ============================================================
# REMOVE INVALID ROWS
# ============================================================

df = df.dropna(
    subset=[
        "D_file",
        WIDTH_COLUMN,
        "ID_shannon_bits",
        "mt_final_entry_s"
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

df["ID_shannon_bits"] = (
    df["ID_shannon_bits"].round(ID_DECIMALS)
)


# ============================================================
# SELECT X-AXIS AND GROUPING
# ============================================================

if MODE == "A":

    # --------------------------------------------------------
    # MT vs A
    # --------------------------------------------------------

    x_column = "D_file"
    x_label = "A (cm)"
    title = "Entry Time vs A"

    # Mean for each A + W
    grouping_columns = [
        "D_file",
        WIDTH_COLUMN
    ]


elif MODE == "ID":

    # --------------------------------------------------------
    # MT vs ID
    # --------------------------------------------------------

    x_column = "ID_shannon_bits"
    x_label = "ID (bits)"
    title = "Entry Time vs ID"

    # Mean for each ID + W
    grouping_columns = [
        "ID_shannon_bits",
        WIDTH_COLUMN
    ]


else:  # MODE == "W"

    # --------------------------------------------------------
    # MT vs W
    # --------------------------------------------------------

    x_column = WIDTH_COLUMN
    x_label = "W"
    title = "Entry Time vs W"

    # Mean for each W + A
    grouping_columns = [
        WIDTH_COLUMN,
        "D_file"
    ]


# ============================================================
# GROUP DATA
# ============================================================

grouped = (
    df.groupby(
        grouping_columns
    )["mt_final_entry_s"]
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
    f"mt_final_entry_s = "
    f"{slope:.6f} × {x_column} + {intercept:.6f}"
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
    # Different colours = different W values

    color_column = WIDTH_COLUMN

elif MODE == "ID":

    # ID mode:
    # Different colours = different A values

    color_column = "D_file"

else:

    # W mode:
    # Different colours = different A values

    color_column = "D_file"


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
        group_data["mt_final_entry_s"],
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
            grouped[WIDTH_COLUMN] == width
        ]

        plt.scatter(
            width_grouped[x_column],
            width_grouped["mean"],
            color="red",
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

    for _, row in grouped.iterrows():

        id_value = row["ID_shannon_bits"]
        width_value = row[WIDTH_COLUMN]

        # Find original A corresponding to this
        # ID + W combination.

        matching_rows = df[
            (df["ID_shannon_bits"] == id_value)
            &
            (df[WIDTH_COLUMN] == width_value)
        ]

        if len(matching_rows) == 0:
            continue

        a_value = matching_rows["D_file"].iloc[0]

        plt.scatter(
            row[x_column],
            row["mean"],
            color="red",
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
            grouped["D_file"] == amplitude
        ]

        plt.scatter(
            amplitude_grouped[x_column],
            amplitude_grouped["mean"],
            color="red",
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
    0.55,
    0.95,
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
plt.ylabel("Entry time (s)")
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
        "_regression_plot_MT_v_A.png"
    )

elif MODE == "ID":

    save_path = csv_path.replace(
        ".csv",
        "_regression_plot_MT_v_ID.png"
    )

else:

    save_path = csv_path.replace(
        ".csv",
        "_regression_plot_MT_v_W.png"
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