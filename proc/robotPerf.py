import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

# ============================================================
# USER SETTINGS
# ============================================================

csv_path = r"C:\Users\NLe\Downloads\M2Fitts_20260911_184728_trials.csv"

# ============================================================
# LOAD DATA
# ============================================================

df = pd.read_csv(csv_path)

# Keep only the columns we need
df = df[["ID_shannon_bits", "mt_final_entry_s"]].copy()

# Convert to numeric
df["ID_shannon_bits"] = pd.to_numeric(df["ID_shannon_bits"], errors="coerce")
df["mt_final_entry_s"] = pd.to_numeric(df["mt_final_entry_s"], errors="coerce")

# Remove rows with missing values
df = df.dropna()

# ============================================================
# GROUP BY ID_SHANNON
# ============================================================

# Mean entry time for each ID
grouped = (
    df.groupby("ID_shannon_bits")["mt_final_entry_s"]
      .agg(["mean", "std", "count"])
      .reset_index()
)

print("\nGrouped data:")
print(grouped)

# ============================================================
# LINEAR REGRESSION
# ============================================================

x = grouped["ID_shannon_bits"].values
y = grouped["mean"].values

result = linregress(x, y)

slope = result.slope
intercept = result.intercept
r_value = result.rvalue
r_squared = r_value ** 2
p_value = result.pvalue
std_err = result.stderr

# Predicted values
y_fit = slope * x + intercept

# ============================================================
# PRINT STATISTICS
# ============================================================

print("\n================ Regression Statistics ================")
print(f"Slope       = {slope:.6f}")
print(f"Intercept   = {intercept:.6f}")
print(f"R           = {r_value:.6f}")
print(f"R²          = {r_squared:.6f}")
print(f"p-value     = {p_value:.6e}")
print(f"Std. Error  = {std_err:.6f}")

print("\nRegression equation:")
print(f"mt_final_entry_s = {slope:.6f} × ID_shannon_bits + {intercept:.6f}")

# ============================================================
# PLOT
# ============================================================

plt.figure(figsize=(10, 6))

# Individual data points
plt.scatter(
    df["ID_shannon_bits"],
    df["mt_final_entry_s"],
    alpha=0.4,
    label="Individual measurements"
)

# Group means
plt.scatter(
    grouped["ID_shannon_bits"],
    grouped["mean"],
    s=60,
    label="Mean per ID"
)

# Regression line
x_line = np.linspace(x.min(), x.max(), 200)
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
    0.95,
    equation_text,
    transform=plt.gca().transAxes,
    verticalalignment="top",
    bbox=dict(boxstyle="round", alpha=0.8)
)

plt.xlabel("ID_shannon_bits")
plt.ylabel("Entry time (s)")
plt.title("Entry Time vs ID")

plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()

plt.show()