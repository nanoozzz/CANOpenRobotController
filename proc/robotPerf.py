import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

# ============================================================
# USER SETTINGS
# ============================================================

csv_path = r"logs/M2Fitts_20260911_190239_trials.csv"

# Number of decimal places used to group similar IDs
ID_DECIMALS = 3

# ============================================================
# LOAD DATA
# ============================================================

df = pd.read_csv(csv_path)

# Keep only the columns we need
df = df[["D_file","ID_shannon_bits", "mt_final_entry_s"]].copy()

# ============================================================
# CONVERT TO NUMERIC
# ============================================================

df["ID_shannon_bits"] = pd.to_numeric(
    df["ID_shannon_bits"],
    errors="coerce"
)

df["mt_final_entry_s"] = pd.to_numeric(
    df["mt_final_entry_s"],
    errors="coerce"
)

df["D_file"] = pd.to_numeric(
    df["D_file"],
    errors="coerce"
) 

# Remove rows with missing/invalid values
df = df.dropna()

# ============================================================
# GROUP SIMILAR IDs
# ============================================================

# Round ID to 3 decimal places BEFORE grouping
#
# Example:
#   3.3217 -> 3.322
#   3.3218 -> 3.322
#   3.3219 -> 3.322
#
# Therefore they are treated as the same ID.

df["ID_shannon_bits"] = df["ID_shannon_bits"].round(ID_DECIMALS)

# ============================================================
# GROUP BY ID
# ============================================================

grouped = (
    #df.groupby("ID_shannon_bits")["mt_final_entry_s"]
    #  .agg(["mean", "std", "count"])
    #  .reset_index()
    df.groupby("D_file")["mt_final_entry_s"]
        .agg(["mean", "std", "count"])
        .reset_index()
)

print("\n================ Grouped Data ================")
print(grouped.to_string(index=False))

# ============================================================
# LINEAR REGRESSION
# ============================================================

#x = grouped["ID_shannon_bits"].values
x = grouped["D_file"].values
y = grouped["mean"].values

# Check that there are enough unique IDs
if len(x) < 2:
    raise ValueError(
        "Not enough unique ID_shannon_bits values for linear regression."
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

print("\n================ Regression Statistics ================")

print(f"Slope       = {slope:.6f}")
print(f"Intercept   = {intercept:.6f}")
print(f"R           = {r_value:.6f}")
print(f"R²          = {r_squared:.6f}")
print(f"p-value     = {p_value:.6e}")
print(f"Std. Error  = {std_err:.6f}")

print("\nRegression equation:")
print(
    f"mt_final_entry_s = "
    f"{slope:.6f} × ID_shannon_bits + {intercept:.6f}"
)

# ============================================================
# PLOT
# ============================================================

plt.figure(figsize=(10, 6))

# ------------------------------------------------------------
# Individual measurements
# ------------------------------------------------------------

plt.scatter(
    #df["ID_shannon_bits"],
    df["D_file"],
    df["mt_final_entry_s"],
    alpha=0.4,
    label="Individual measurements"
)

# ------------------------------------------------------------
# Group means
# ------------------------------------------------------------

plt.scatter(
    #grouped["ID_shannon_bits"],
    grouped["D_file"],
    grouped["mean"],
    s=70,
    label="Mean per ID"
)

# ------------------------------------------------------------
# Regression line
# ------------------------------------------------------------

x_line = np.linspace(
    x.min(),
    x.max(),
    200
)

y_line = slope * x_line + intercept

plt.plot(
    x_line,
    y_line,
    linewidth=2,
    label="Linear regression"
)

# ============================================================
# EQUATION + STATISTICS ON PLOT
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

#plt.xlabel("ID Shannon (bits)")
plt.xlabel("A (cm)")
plt.ylabel("Entry time (s)")
plt.title("Entry Time vs ID Shannon")

plt.grid(True, alpha=0.3)
plt.legend()

plt.tight_layout()
plt.show()

save_path = csv_path.replace(".csv", "_regression_plot_MT_v_A.png")
#save_path = csv_path.replace(".csv", "_regression_plot_MT_v_ID.png")
plt.savefig(save_path, dpi=300)