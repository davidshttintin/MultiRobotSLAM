import numpy as np

# grid is a 2d array, start and end are both tuples
def lee_planning_path(grid, start, end):
    status = np.zeros((len(grid), len(grid[0])))
    visited = np.zeros((len(grid), len(grid[0])))
    queue = []
    queue.append((start, 0))
    while len(queue) > 0:
        (i, j), val = queue.pop(0)
        if visited[i][j] == 1:
            continue
        visited[i][j] = 1
        if (i, j) == end:
            status[i][j] = val
            break
        if grid[i][j] < 10:
            status[i][j] = -1
            continue
        status[i][j] = val
        left = (i-1) >= 0
        right = (i+1) < len(grid)
        top  = (j-1) >= 0
        bottom = (j+1) < len(grid[0])
        if left:
            queue.append(((i-1, j), val+1))
        if right:
            queue.append(((i+1, j), val+1))
        if top:
            queue.append(((i, j-1), val+1))
        if bottom:
            queue.append(((i, j+1), val+1))
        #if left and top:
            #queue.append(((i-1, j-1), val+1))
        #if left and bottom:
            #queue.append(((i-1, j+1), val+1))
        #if right and top:
            #queue.append(((i+1, j-1), val+1))
        #if right and bottom:
            #queue.append(((i+1, j+1), val+1))
    #print(status)
    if status[end[0]][end[1]] == 0 and start != end:
        print("not accessible")
        return [] #end is not accessible
    path = []
    path.append(end)
    while True:
        i, j = path[0]
        if (i, j) == start:
            break
        min_val = 999
        min_xy = (0, 0)
        left = (i-1) >= 0 and status[i-1][j] != -1 and visited[i-1][j] == 1
        right = (i+1) < len(grid) and status[i+1][j] != -1 and visited[i+1][j] == 1
        top  = (j-1) >= 0 and status[i][j-1] != -1 and visited[i][j-1] == 1
        bottom = (j+1) < len(grid[0]) and status[i][j+1] != -1 and visited[i][j+1] == 1
        if left:
            if status[i-1][j] < min_val:
                min_val = status[i-1][j]
                min_xy = (i-1, j)
        if right:
            if status[i+1][j] < min_val:
                min_val = status[i+1][j]
                min_xy = (i+1, j)
        if top:
            if status[i][j-1] < min_val:
                min_val = status[i][j-1]
                min_xy = (i, j-1)
        if bottom:
            if status[i][j+1] < min_val:
                min_val = status[i][j+1]
                min_xy = (i, j+1)
        path.insert(0, min_xy)
    #print(path)
    waypoints = find_waypoints(path)
    print(waypoints)
    return waypoints

def find_waypoints(path):
    waypoints = []
    for idx, pt in enumerate(path):
        if idx == 0:
            continue
        if idx == len(path) - 1:
            waypoints.append(pt)
            break
        diff = np.array(path[idx + 1]) - np.array(path[idx - 1])
        if (np.abs(diff) == np.array([1, 1])).all():
            waypoints.append(pt)
    return waypoints

a = np.array([[10, 20, 30, 0, 100],
              [0, 10, 40, 50, 10],
              [0, 20, 50, 0, 100],
              [10, 50, 60, 70, 80]])
print(a)
lee_planning_path(a, (0, 0), (3, 4))
