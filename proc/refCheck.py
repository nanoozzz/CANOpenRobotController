import pandas as pd

# =========================
# CSV file path
# =========================
#csv_file = r"C:\Users\YourName\Documents\data.csv"

# For Linux/macOS, for example:
csv_file = "schedule/bal_group_4.csv"


# =========================
# Read CSV
# =========================
df = pd.read_csv(csv_file)

# Check required columns
required_columns = {"index", "A", "W", "ID"}

if not required_columns.issubset(df.columns):
    raise ValueError(
        f"CSV must contain columns: {required_columns}"
    )


# =========================
# Count each ID
# =========================
id_counts = df["ID"].value_counts().sort_index()


# =========================
# Count each (A, W) pair
# =========================
aw_counts = (
    df.groupby(["A", "W"])
    .size()
    .reset_index(name="count")
    .sort_values(["A", "W"])
)


# =========================
# Print results
# =========================
print("\n=== ID Counts ===")
print(id_counts.to_string())

print("\n=== (A, W) Pair Counts ===")
print(aw_counts.to_string(index=False))