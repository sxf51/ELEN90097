import numpy as np
import pandas as pd
import sqlite3
import seaborn as sns
import matplotlib.pyplot as plt

def load_company_data(file_path, company_name):
    df = pd.read_csv(file_path)
    df_melted = df.melt(id_vars=["Date"], var_name="Time", value_name="generation_mw")
    df_melted["timestamp"] = pd.to_datetime(df_melted["Date"] + " " + df_melted["Time"], format="%d/%m/%Y %H:%M:%S")
    df_melted["company"] = company_name
    return df_melted[["timestamp", "company", "generation_mw"]]

files = {
    "50 Hertz": "50Hertz.csv",
    "Amprion": "Amprion.csv",
    "TenneT TSO": "TenneTTSO.csv",
    "TransnetBW": "TransnetBW.csv"
}

df_all = pd.concat([load_company_data(path, name) for name, path in files.items()], ignore_index=True)
df_all.to_csv("combined_data.csv", index=False)

conn = sqlite3.connect("wind_power.db")
df_all.to_sql("generation_data", conn, if_exists="replace", index=False)

# ------------------------------------------------------
# ------------------------------------------------------
query_covid_pre = """
SELECT company, AVG(generation_mw) as avg_mw
FROM generation_data
WHERE timestamp BETWEEN '2019-09-01' AND '2020-03-01'
GROUP BY company;
"""

query_covid_post = """
SELECT company, AVG(generation_mw) as avg_mw
FROM generation_data
WHERE timestamp > '2020-03-01'
GROUP BY company;
"""

df_pre = pd.read_sql_query(query_covid_pre, conn)
df_post = pd.read_sql_query(query_covid_post, conn)

print("=== 疫情前发电均值 ===")
print(df_pre)
print("\n=== 疫情后发电均值 ===")
print(df_post)


df_pre["period"] = "Pre-COVID"
df_post["period"] = "Post-COVID"


df_covid = pd.concat([df_pre, df_post])


plt.figure(figsize=(10, 6))
sns.barplot(data=df_covid, x="company", y="avg_mw", hue="period")
plt.title("Average Wind Generation Before and After COVID")
plt.ylabel("Average Generation (TWh)")
plt.xlabel("Company")
plt.xticks(rotation=15)
plt.tight_layout()
plt.show()

# ------------------------------------------------------
# ------------------------------------------------------
query_day_night_daily = """
WITH daily_avg AS (
    SELECT 
        company,
        DATE(timestamp) as date,
        CASE 
            WHEN CAST(strftime('%H', timestamp) AS INTEGER) BETWEEN 6 AND 18 THEN 'day'
            ELSE 'night'
        END AS period,
        SUM(generation_mw) AS total_tw_daily
    FROM generation_data
    GROUP BY company, date, period
)
SELECT 
    company,
    period,
    AVG(total_tw_daily) AS avg_daily_tw
FROM daily_avg
GROUP BY company, period
ORDER BY company, period;
"""

df_day_night_daily = pd.read_sql_query(query_day_night_daily, conn)
print("\n=== 白天 vs 夜间 发电量 ===")
print(df_day_night_daily)



# ------------------------------------------------------
# ------------------------------------------------------
query_extreme = """
SELECT company, timestamp, generation_mw
FROM generation_data
WHERE generation_mw > 700
ORDER BY generation_mw DESC
LIMIT 10;
"""

df_extreme = pd.read_sql_query(query_extreme, conn)
print("\n=== 极端高发电时段 (Top 10) ===")
print(df_extreme)

conn.close()

summary = df_all.groupby("company")["generation_mw"].agg(["mean", "median", "std"])
print(summary)

df_all["date"] = df_all["timestamp"].dt.date
daily_avg = df_all.groupby(["date", "company"])["generation_mw"].mean().reset_index()

for company in daily_avg["company"].unique():
    subset = daily_avg[daily_avg["company"] == company]
    plt.plot(subset["date"], subset["generation_mw"], label=company)

plt.title("Average daily wind power")
plt.xlabel("Data")
plt.ylabel("Generation of electricity (TWh)")
plt.legend()
plt.tight_layout()
plt.show()

sns.boxplot(x="company", y="generation_mw", data=df_all)
plt.title("Comparison of power generation distribution by company")
plt.ylabel("15-minute power generation (TWh)")
plt.show()


conn = sqlite3.connect("wind_power.db")
df = pd.read_sql_query("SELECT * FROM generation_data", conn)
conn.close()

df["timestamp"] = pd.to_datetime(df["timestamp"])
df = df.set_index("timestamp")

company_name = "50 Hertz"
df_company = df[df["company"] == company_name]


weekend_flag = np.where(df_company.index.weekday < 5, "Weekday", "Weekend")

by_time = df_company.groupby([weekend_flag, df_company.index.time]).mean(numeric_only=True)

fig, ax = plt.subplots(1, 2, figsize=(14, 5), sharey=True)
hourly_ticks = [pd.to_datetime(f"{h:02}:00").time() for h in range(0, 24, 2)]

by_time.loc["Weekday"]["generation_mw"].plot(ax=ax[0], title="Weekday", xticks=hourly_ticks)
by_time.loc["Weekend"]["generation_mw"].plot(ax=ax[1], title="Weekend", xticks=hourly_ticks)

for a in ax:
    a.set_xlabel("Time of Day")
    a.set_ylabel("Average Generation (TWh)")

plt.suptitle(f"Comparison of Intraday Trends in Wind Power (company:{company_name})")
plt.tight_layout()
plt.show()