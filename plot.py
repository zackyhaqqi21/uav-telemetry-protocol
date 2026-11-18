"""Plot results of ./bench (sweep.csv, per_fault.csv).  Usage: python3 plot.py"""
import csv

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


sweep = read("sweep.csv")
x = [float(r["level_pct"]) for r in sweep]
mean = [float(r["delivery_mean"]) for r in sweep]
std = [float(r["delivery_std"]) for r in sweep]
lost_est = [float(r["lost_est"]) for r in sweep]
lost_true = [float(r["lost_true"]) for r in sweep]

fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.2))

a1.plot(x, mean, marker="o", color="#1f77b4")
a1.fill_between(x, [m - s for m, s in zip(mean, std)],
                [m + s for m, s in zip(mean, std)], alpha=0.25, color="#1f77b4")
a1.set_xlabel("Noise level (% of packets hit by each fault type)")
a1.set_ylabel("Packets delivered intact (%)")
a1.set_title("Parser robustness vs link noise")
a1.set_ylim(0, 102)
a1.grid(alpha=0.3)

a2.plot(x, lost_true, marker="o", label="actually lost (ground truth)")
a2.plot(x, lost_est, marker="s", label="parser's packets_lost estimate")
a2.set_xlabel("Noise level (% of packets hit by each fault type)")
a2.set_ylabel("Packets (per 1000 sent)")
a2.set_title("Loss estimate from SEQ gaps vs reality")
a2.legend()
a2.grid(alpha=0.3)

fig.suptitle("uav-telemetry-protocol: 1000 packets/trial, 30 trials/point, error band = 1 std",
             fontsize=9)
fig.tight_layout()
fig.savefig("results.png", dpi=150)

pf = read("per_fault.csv")
names = [r["fault"] for r in pf]
vals = [float(r["delivery_mean"]) for r in pf]
fig2, ax = plt.subplots(figsize=(6.5, 4))
bars = ax.bar(names, vals, color="#2ca02c")
ax.bar_label(bars, fmt="%.1f%%")
ax.set_ylabel("Packets delivered intact (%)")
ax.set_title("Which fault hurts most? (each fault hits ~10% of packets)")
ax.set_ylim(0, 105)
ax.grid(axis="y", alpha=0.3)
fig2.tight_layout()
fig2.savefig("per_fault.png", dpi=150)
print("wrote results.png, per_fault.png")
