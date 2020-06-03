import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import time


class PathFindObject:
    def __init__(self, x, y, cost = 1):
        self.x = x
        self.y = y
        self.cost = cost
        self.camefrom = None
        self.gscore = np.inf
        self.fscore = np.inf

    @property
    def position(self):
        return np.array([self.x, self.y])

    def __call__(self):
        return np.array((self.x,self.y))


def heuristic(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)


def reconstruct_path(target):
    path = [target.position]
    current = target
    while current.camefrom is not None:
        path.append(current.camefrom.position)
        current = current.camefrom

    return np.array(path)[::-1]


def find_path(start, target, map_grid):
    target_position = target.position
    start_position = start.position

    start.camefrom = None
    start.gscore = 0
    start.fscore = heuristic(start_position, target_position)

    print(start.fscore)

    open_set = [start]
    open_set_fscores = [start.fscore]
    closed_set = []

    map_size = map_grid.shape

    while len(open_set) > 0:
        """
        print("[", end = "")
        for item in open_set:
            print(f"({item.x}, {item.y})", end = "")
        print("]")"""
        current = open_set[0]

        if current == target:
            """ Target reached """
            for item in open_set + closed_set:
                item.fscore = np.inf
                item.gscore = np.inf
            return reconstruct_path(target)
        
        open_set.pop(0)
        open_set_fscores.pop(0)

        closed_set.append(current)

        current_pos = current.position
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == j == 0:
                    continue
                
                other_x = current_pos[0] + j
                other_y = current_pos[1] + i

                if other_x < 0 or other_x >= map_size[1]:
                    continue
                if other_y < 0 or other_y >= map_size[0]:
                    continue
                
                neighbor = map_grid[other_y, other_x]
                neighbor_pos = neighbor.position

                tentative_gscore = current.gscore + np.linalg.norm([i, j])*neighbor.cost

                if tentative_gscore < neighbor.gscore:
                    neighbor.camefrom = current
                    neighbor.gscore = tentative_gscore
                    neighbor.fscore = tentative_gscore + heuristic(neighbor_pos, target_position)

                    if neighbor not in open_set:
                        open_set.append(neighbor)
                        open_set_fscores.append(neighbor.fscore)
                    else:
                        ind = open_set.index(neighbor)
                        open_set_fscores[ind] = neighbor.fscore
        

def plot_path(path, linestyle = "-", color = "black"):
    path = np.array(path)
    #path[:,1] = len(path[:,1]) - path[:,1]
    plt.plot(path[0][0], path[0][1], marker = "x", color = "red")
    plt.plot(path[-1][0], path[-1][1], marker = "x", color = "blue")
    current_pos = path[0]
    for pos in path[1:]:
        plt.plot([current_pos[0], pos[0]], [current_pos[1], pos[1]], color = color, linestyle = linestyle)
        current_pos = pos

    plt.axis("equal")


if __name__ == "__main__":
    with open("maptxt_1.txt", "r") as infile:
        lines = infile.readlines()
    height = len(lines)
    width = len(lines[0].strip())

    grid = np.zeros((height, width))

    print(grid.shape)

    for i, row in enumerate(lines):
        for j, item in enumerate(row.strip()):
            if item == "1":
                grid[i,j] = 1

    grid = grid.T

    plt.imshow(grid)

    map_grid = np.empty(shape = grid.shape, dtype = PathFindObject)
    for i, row in enumerate(grid):
        for j, val in enumerate(row):
            if val == 1:
                map_grid[i, j] = PathFindObject(j, i, np.inf)
            else:
                map_grid[i, j] = PathFindObject(j, i)

    start = map_grid[16, 24]
    target = map_grid[0, 0]

    path = find_path(start, target, map_grid)
    plot_path(path, color = "red", linestyle = ":")

    cpp_path = np.array([[24, 16],[25, 17],[26, 18],[26, 19],[26, 20],[27, 21],[28, 22],[27, 23],[26, 24],[26, 25],[27, 26],[28, 27],[28, 28],[29, 29],[30, 28],[30, 27],[30, 26],[30, 25],[30, 24],[30, 23],[30, 22],[30, 21],[30, 20],[30, 19],[29, 18],[28, 17],[28, 16],[28, 15],[28, 14],[28, 13],[28, 12],[28, 11],[27, 10],[26, 10],[25, 10],[24, 9],[25, 8],[26, 8],[27, 8],[28, 7],[29, 7],[30, 7],[31, 6],[30, 5],[29, 4],[28, 3],[27, 2],[26, 1],[25, 0],[24, 0],[23, 0],[22, 0],[21, 0],[20, 0],[19, 0],[18, 0],[17, 0],[16, 0],[15, 0],[14, 0],[13, 0],[12, 0],[11, 0],[10, 0],[9, 0],[8, 0],[7, 0],[6, 0],[5, 0],[4, 0],[3, 0],[2, 0],[1, 0],[0, 0],])
    plot_path(cpp_path, color = "green", linestyle = "-")

    plt.show()

