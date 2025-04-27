import matplotlib.pyplot as plt

colores = {
    "Complementary Filter": "red",
    "Euler Direct": "blue",
    "Kalman Filter": "green",
    "Fusion Simple": "orange",
    "Fusion Advanced": "magenta",
    "Fusion Advanced Calibration": "cyan",
}

fig, ax = plt.subplots()

for label, color in colores.items():
    ax.plot([], [], color=color, label=label)

ax.legend()
ax.axis("off")
plt.show()
