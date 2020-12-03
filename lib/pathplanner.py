import numpy as np
import time

def lee_planning_path(grid, start, end):
    status = np.zeros((len(a), len(grid[0])))
    can_visit = np.ones((len(a), len(grid[0])))
    visited = np.zeros((len(a), len(grid[0])))

    reached = False
    status[grid < 10] = -1 #set obstacles to -1
    can_visit[grid < 10] = 0
    status[start] = 1
    can_visit[start] = 0
    current_step = 1
    visited[start] = 1
    while not reached:
        current_step_map = status == current_step
        rollup = np.roll(current_step_map, -1, axis = 0)
        rollup[-1] = False
        rollup = np.logical_and(rollup, can_visit)

        rolldown = np.roll(current_step_map, 1, axis = 0)
        rolldown[0] = False
        rolldown = np.logical_and(rolldown, can_visit)

        rollright = np.roll(current_step_map, 1, axis = 1)
        rollright[:, 0] = False
        rollright = np.logical_and(rollright, can_visit)

        rollleft = np.roll(current_step_map, -1, axis = 1)
        rollleft[:, -1] = False
        rollleft = np.logical_and(rollleft, can_visit)

        this_step = np.logical_or(np.logical_or(rollup, rolldown), np.logical_or(rollright, rollleft))
        assert(not np.sum(np.logical_and(this_step, np.logical_not(can_visit))))

        can_visit = np.logical_and(np.logical_not(this_step), can_visit)
        visited = np.logical_or(this_step, visited)

        status[this_step] = current_step+1
        current_step+=1

        coordinates = np.transpose(np.nonzero(this_step))
        reached = list(end) in coordinates.tolist()
        
        
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
