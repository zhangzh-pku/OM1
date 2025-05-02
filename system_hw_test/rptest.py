from rpdriver import RPDriver
import time
import sys
import numpy as np

from matplotlib import pyplot as plot
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from numpy import array, linspace
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks



# center = ax[0].plot([0], [0], "o", color="blue")[0]  # the robot
# circle = ax[0].add_patch(Circle((0, 0), 0.16, color="red"))
# line = ax[0].plot([], [], ".", color="black")[0]
# front = ax[0].annotate("Front", xytext=(0.1, 0.3), xy=(0, 0.5))
# arrow = ax[0].annotate("", xytext=(0, 0), xy=(0, 0.5), arrowprops=dict(arrowstyle="->"))
# ax[0].set_xlim(-1, 1)
# ax[0].set_ylim(-1, 1)
# ax[0].set_aspect("equal")

# # Add captions to Fig2 to to orient people
# gap = 5
# ax[1].plot([0, 227 - gap], [50, 50], "-", color="red", linewidth=3.0)[0]
# ax[1].plot([227 + gap, 453 - gap], [50, 50], "-", color="red", linewidth=3.0)[0]
# ax[1].plot([453 + gap, 680], [50, 50], "-", color="red", linewidth=3.0)[0]
# ax[1].annotate("Left", xytext=(100, 45), xy=(0, 0.5))
# ax[1].annotate("Front", xytext=(320, 45), xy=(0, 0.5))
# ax[1].annotate("Right", xytext=(550, 45), xy=(0, 0.5))
# arrow_list = []

distances = zeros(361)
# angles = list(range(361))

PORT_NAME = '' # Adjust the port according to your system

if len(sys.argv) > 1:
    PORT_NAME = sys.argv[1]
    print(f"using {PORT_NAME} as the serial port")
else:
    print("you forgot to specify the serial port...")


fig, ax = plot.subplots(2)
line = ax[1].plot([], [], ".", color="red")[0]
#line2 = ax[1].plot([], [], ".", color="green")[0]
ax[1].set_xlim(0, 680)
ax[1].set_ylim(-1, 51)  # cm
#ani = FuncAnimation(fig, lambda _: None)

lidar = RPDriver(PORT_NAME)
info = lidar.get_info()
print(f"Info: {info}")

health = lidar.get_health()
print(f"Health: {health}")

# reset to clear buffers
lidar.reset()
#lidar.start("express")

#for i, scan in enumerate(lidar.iter_measures()):



# # for i, scan in enumerate(lidar.iter_scans("normal", 3000, 300)):
for i, scan in enumerate(lidar.iter_scans(scan_type='express', max_buf_meas=3000, min_len=5)):

    print(f"{i}: {scan}")
    print(f"{scan[0][0]}")
    
    array = np.array(scan)
    print(f"{len(array[:,1])}")

    angles = array[:,1]
    distances = array[:,2]

    line.set_data(angles, distances)
    plot.show()

        # print(f"{int(array[:,1])}")

    #     #if i > 10:
    #     #    break

    # lidar.stop()
    # #lidar.stop_motor()
    # lidar.disconnect()

# except Exception as e:
#      print(f"An error occurred: {e}")

# # finally:
# #     if 'lidar' in locals():
# #         #lidar.stop()
# #         #lidar.stop_motor()
# #         #lidar.disconnect()