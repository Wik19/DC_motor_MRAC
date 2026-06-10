#!/usr/bin/env python3
"""Plot the simulation log. Usage: python plot.py [closedloop.csv]"""
import sys
import numpy as np
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "closedloop.csv"
d = np.genfromtxt(path, delimiter=",", names=True)

fig, ax = plt.subplots(4, 1, figsize=(9, 10), sharex=True)

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

fig.tight_layout()
out = path.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=110)
print("wrote", out)
if "--show" in sys.argv:
    plt.show()
