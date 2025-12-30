import csv
import os
import matplotlib.pyplot as plt

csv_path = "/home/jhuo/robstride_usb2can_ctrl/logs/track.csv"

if not os.path.exists(csv_path):
    raise FileNotFoundError(csv_path)

# 按电机编号存数据
data = {}  # i -> dict of lists

with open(csv_path, "r", newline="", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    for row in reader:
        try:
            i = int(row["i"])
            d = data.setdefault(i, {"t": [], "q": [], "dq": [], "q_des": [], "dq_des": []})
            d["t"].append(float(row["t"]))
            d["q"].append(float(row["q"]))
            d["dq"].append(float(row["dq"]))
            d["q_des"].append(float(row["q_des"]))
            d["dq_des"].append(float(row["dq_des"]))
        except Exception:
            continue

motor_ids = sorted(data.keys())
print("found motors:", motor_ids)

# 你要求“六个电机都要”
if len(motor_ids) < 6:
    print("[warn] motors < 6, will plot what we have.")
motor_ids = motor_ids[:6]  # 只画前6个（通常就是0~5）

def plot_grid(title, ykey, yrefkey, ylabel):
    fig, axes = plt.subplots(3, 2, sharex=True)
    fig.suptitle(title)

    axes = axes.flatten()
    for idx, mid in enumerate(motor_ids):
        ax = axes[idx]
        d = data[mid]
        ax.plot(d["t"], d[ykey], label=ykey)
        ax.plot(d["t"], d[yrefkey], label=yrefkey)
        ax.set_title(f"Motor {mid}")
        ax.set_ylabel(ylabel)
        ax.grid(True)
        ax.legend(fontsize=8)

    # 最后一行加 x 标签
    axes[4].set_xlabel("t (s)")
    axes[5].set_xlabel("t (s)")
    plt.tight_layout()
    return fig

plot_grid("Position tracking (q vs q_des)", "q", "q_des", "q (rad)")
plot_grid("Velocity tracking (dq vs dq_des)", "dq", "dq_des", "dq (rad/s)")

plt.show()
