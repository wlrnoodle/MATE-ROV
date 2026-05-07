import matplotlib.pyplot as plt
import numpy as np

# fake dive data simulating two profiles
t1 = np.linspace(0, 60, 300)
t2 = np.linspace(65, 125, 300)

# profile 1 — descend to 2.5m, hold, come back to 0.4m
def fake_profile(t_start):
    t = np.linspace(0, 60, 300)
    depth = np.where(t < 20, t * (2.5/20),           # descend
            np.where(t < 35, 2.5 + np.random.normal(0, 0.02, 300),  # hold 2.5m
            np.where(t < 50, 2.5 - (t - 35) * (2.1/15),             # ascend
                     0.4 + np.random.normal(0, 0.01, 300))))          # hold 0.4m
    return t + t_start, depth

t1, d1 = fake_profile(0)
t2, d2 = fake_profile(65)

fig, ax = plt.subplots(figsize=(11, 5))

ax.plot(t1, d1, color="steelblue", linewidth=1.8, label="Profile 1")
ax.plot(t2, d2, color="coral",     linewidth=1.8, label="Profile 2")

ax.axhline(2.5,  color="gray", linestyle="--", linewidth=0.8, label="2.5 m target")
ax.axhline(0.40, color="red",  linestyle="--", linewidth=0.8, label="40 cm ceiling")
ax.axhspan(2.4,  2.6,  alpha=0.08, color="steelblue")
ax.axhspan(0.35, 0.45, alpha=0.08, color="red")

ax.invert_yaxis()
ax.set_xlabel("Time (s)")
ax.set_ylabel("Depth (m)")
ax.set_title("Vertical profiles — depth over time")
ax.legend()
fig.tight_layout()
fig.savefig("/home/programming-pathway/MATE-ROV/Graph_Data/graph_test.png", dpi=150)
print("saved → /home/programming-pathway/MATE-ROV/graph_test.png")
plt.show()
