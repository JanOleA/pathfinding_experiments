import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation, cm
import seaborn as sns

with open("output.txt", "r") as infile:
    lines = infile.readlines()

current_pos = []

for i, line in enumerate(lines):
    line = line.split(";")[:-1]
    cpos = np.array(line[-1].strip("[]").split(","), dtype = np.float64)
    current_pos.append(cpos + 0.5)
    line = line[:-1]
    for j, item in enumerate(line):
        line[j] = np.array(item.strip("[]").split(","), dtype = np.float64)
    lines[i] = np.array(line)

lines = np.array(lines)
mapsize = (32,32)

frame = 0
x = lines[frame,:,0]
y = lines[frame,:,1]
cf_x = lines[frame,:,2]
cf_y = lines[frame,:,3]
gscores = lines[frame,:,4]
fscores = lines[frame,:,5]
costs = lines[frame,:,6]
open_set = lines[frame,:,7]

fig = plt.figure(figsize = (12,12))

square_gscores = np.reshape(gscores, mapsize)
square_costs = np.reshape(costs, mapsize)

square_costs[square_costs == np.inf] = 0

ax2 = sns.heatmap(square_costs, square=True, cbar=False, vmax = 1, vmin = 0)
ax = sns.heatmap(square_gscores, square=True, cbar=False, annot=True, vmax = 100, cmap = cm.winter)
openset_x = x[open_set == 1] + 0.5
openset_y = y[open_set == 1] + 0.5
scatr = plt.scatter(openset_x, openset_y, c = "red", alpha = 0.5, s = 50)
curmarker = plt.plot(current_pos[frame][0], current_pos[frame][1], marker = (4, 0, 0), color = "green", markersize = 10)


def init():
    plt.clf()
    ax2 = sns.heatmap(square_costs, square=True, cbar=False, vmax = 1, vmin = 0)
    ax = sns.heatmap(square_gscores, square=True, cbar=False, annot=True, vmax = 100, cmap = cm.winter)
    scatr = plt.scatter(openset_x, openset_y, c = "red", alpha = 0.5, s = 50)
    curmarker = plt.plot(current_pos[frame][0], current_pos[frame][1], marker = (4, 0, 0), color = "green", markersize = 10)


def animate(frame):
    print(f"{frame/len(lines)*100:2.2f}   ", end = "\r")
    plt.clf()
    x = lines[frame,:,0]
    y = lines[frame,:,1]
    cf_x = lines[frame,:,2]
    cf_y = lines[frame,:,3]
    gscores = lines[frame,:,4]
    fscores = lines[frame,:,5]
    costs = lines[frame,:,6]
    open_set = lines[frame,:,7]
    square_gscores = np.reshape(gscores, mapsize)
    ax2 = sns.heatmap(square_costs, square=True, cbar=False, vmax = 1, vmin = 0)
    ax = sns.heatmap(square_gscores, square=True, cbar=False, annot=True, vmax = 100, cmap = cm.winter, fmt = "2.2f", annot_kws = {"size": 6})
    openset_x = x[open_set == 1] + 0.5
    openset_y = y[open_set == 1] + 0.5
    scatr = plt.scatter(openset_x, openset_y, c = "red", alpha = 0.5, s = 50)
    curmarker = plt.plot(current_pos[frame][0], current_pos[frame][1], marker = (4, 0, 0), color = "green", markersize = 10)
    plt.savefig(f"output/{frame}.png")

save_anim = True
Writer = animation.writers['ffmpeg']
writer = Writer(fps=10, metadata=dict(artist='Me'), bitrate=3000)

anim = animation.FuncAnimation(fig, animate, init_func = init, frames=len(lines), interval=10)

if save_anim:
    print("Saving animation...")
    anim.save(f"output/movie.mp4", writer=writer)
plt.show()