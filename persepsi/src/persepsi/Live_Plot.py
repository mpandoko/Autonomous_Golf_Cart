from dataclasses import dataclass
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


gridsize = (8, 2)
fig = plt.figure(figsize=(6, 6))
ax1 = plt.subplot2grid(gridsize, (0,0), colspan=2, rowspan=5)
ax2 = plt.subplot2grid(gridsize, (5,0), colspan=1, rowspan=2)
ax3 = plt.subplot2grid(gridsize, (5,1), colspan=1, rowspan=2)

# Initialize camera position
ax1.plot(0, 'r*', markersize=10, label = 'Camera')

def animate(i):
    try:
        save_dir = "inference/output/r.txt"
        data = pd.read_csv(save_dir, sep=" ", header=None)
        data = data.drop([7, 8, 9, 10], axis=1)        
        data.columns = ["frame", "id", "x", "z", "vx", "vz", "time"]
        datax = data.tail(20)
        x0 = {}
        z0 = {}
        sbx0 = {}
        vx0 = {}
        vz0 = {}
        ids = datax.id.unique()
        ids = ids[:3] #pick top 3
        for u_id in ids:
            data_u = datax[datax.id == u_id]
            x0.update({u_id : data_u[["x"]].to_numpy().reshape(-1, 1)})
            z0.update({u_id : data_u[["z"]].to_numpy().reshape(-1, 1)})
            vx0.update({u_id : data_u[["vx"]].to_numpy().reshape(-1, 1)})
            vz0.update({u_id: data_u[["vz"]].to_numpy().reshape(-1, 1)})
            sbx0.update({u_id : data_u[["time"]].to_numpy().reshape(-1, 1)})

        tabs = ['tab:blue', 'tab:red', 'tab:green', 'tab:grey']
        brg = ['b', 'r', 'g', 'gray']
        ax1.cla()
        ax2.cla()
        ax3.cla()

        for i, u_id in enumerate(ids):
            ax1.plot(x0[u_id], z0[u_id], brg[i], label='Object ' + str(u_id), lw=2)
            ax2.plot(sbx0[u_id], vx0[u_id], tabs[i], label = "Object " + str(u_id))
            ax3.plot(sbx0[u_id], vz0[u_id], tabs[i], label = "Object " + str(u_id))
        ax1.set(xlim=(-1, 1), ylim=(0, 5))
        ax1.set_xlabel('x (meter)')
        ax1.set_ylabel('z (meter)')
        ax1.set_title('Trajectory')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('(m/s)')
        ax2.set_title('Velocity x')
        ax3.set_xlabel('Time (s)')
        ax3.set_title('Velocity z')
        ax1.legend(loc = "upper left")
        ax2.legend(loc = "upper left")
        ax3.legend(loc = "upper left")


    except:
        # print("NO FILE DETECTED ...")
        pass

# Call animation function
anim = FuncAnimation(fig, animate, interval = 20, repeat_delay = 5)


# Show the plot
plt.tight_layout()
plt.show()