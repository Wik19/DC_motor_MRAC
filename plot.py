#!/usr/bin/env python3
"""Plot a simulation log. Usage: python plot.py [file.csv] [--show]"""
import sys
import numpy as np
import matplotlib.pyplot as plt

args = [a for a in sys.argv[1:] if not a.startswith("--")]
path = args[0] if args else "mrac.csv"
d = np.genfromtxt(path, delimiter=",", names=True)
names = d.dtype.names

fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)

if "kx1" in names:  # MRAC log
    ax[0].plot(d["t"], d["r"], color="0.7", drawstyle="steps-post",
               label="command r")
    ax[0].plot(d["t"], d["y_m"], "--", label="reference model y_m")
    ax[0].plot(d["t"], d["theta_m"], label="motor theta_m")
    ax[0].set_ylabel("position [rad]")
    ax[0].legend(loc="upper right")

    ax[1].plot(d["t"], 1e3 * d["e1"])
    ax[1].set_ylabel("tracking error e1 [mrad]")

    ax[2].plot(d["t"], d["kx1"], label="kx1")
    ax[2].plot(d["t"], d["kx2"], label="kx2")
    ax[2].plot(d["t"], d["kr"], label="kr")
    ax[2].set_ylabel("adaptive gains")
    ax[2].legend(loc="right")

    ax[3].plot(d["t"], d["u_a"])
    ax[3].set_ylabel("voltage u_a [V]")
    ax[3].set_xlabel("time [s]")
else:  # baseline log
    ax[0].plot(d["t"], d["theta_md"], "--", label="reference")
    ax[0].plot(d["t"], d["theta_m"], label="motor")
    ax[0].set_ylabel("position [rad]")
    ax[0].legend(loc="upper right")

    ax[1].plot(d["t"], 1e3 * d["e1"])
    ax[1].set_ylabel("pos. error [mrad]")

    ax[2].plot(d["t"], d["omega_md"], "--", label="reference")
    ax[2].plot(d["t"], d["omega_m"], label="true")
    ax[2].plot(d["t"], d["omega_est"], ":", label="estimated")
    ax[2].set_ylabel("velocity [rad/s]")
    ax[2].legend(loc="upper right")

    ax[3].plot(d["t"], d["u_a"], label="u_a")
    ax[3].plot(d["t"], d["u_ff"], "--", label="feedforward")
    ax[3].set_ylabel("voltage [V]")
    ax[3].set_xlabel("time [s]")
    ax[3].legend(loc="upper right")

for a in ax:
    a.grid(True, which="both", linestyle=":", color="0.7")

fig.tight_layout()
out = path.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=110)
print("wrote", out)
if "--show" in sys.argv:
    plt.show()
