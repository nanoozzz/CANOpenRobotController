import pandas as pd

# =========================
# Configuration
# =========================

input_csv = "../schedule/bal_group_all.csv"
output_csv = "../schedule/bal_group_all_alpha.csv" # change P for different participants.

DEFAULT_ALPHA = 0.5


# =========================
# Read CSV
# =========================

df = pd.read_csv(input_csv)


# =========================
# Add alpha column
# =========================

df["alpha"] = DEFAULT_ALPHA


# =========================
# Multiple alpha logic
# =========================
#
# Example: assign different alpha values based on a condition.
#
# df["alpha"] = 0.2
# df.loc[df["ID"] < 3, "alpha"] = 0.1
# df.loc[df["ID"] >= 3, "alpha"] = 0.3
#
# Or, for example, based on another column:
#
# alpha_map = {
#     1: 0.1,
#     2: 0.2,
#     3: 0.3,
# }
# df["alpha"] = df["condition"].map(alpha_map)
#
# =========================


# =========================
# Save CSV
# =========================

df.to_csv(output_csv, index=False)

print(f"Saved modified CSV to: {output_csv}")