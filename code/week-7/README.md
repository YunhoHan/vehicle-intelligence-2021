# Week 7 - Hybrid A* Algorithm & Trajectory Generation

---
[//]: # (Image References)
[result1]: ./speed_0.5_NUM_THETA_CELLS_90.png
[result2]: ./speed_1.0_NUM_THETA_CELLS_90.png
[result3]: ./speed_0.5_NUM_THETA_CELLS_180.png
[result4]: ./speed_1.0_NUM_THETA_CELLS_180.png
[result5]: ./speed_0.5_NUM_THETA_CELLS_360.png
[result6]: ./speed_1.0_NUM_THETA_CELLS_360.png
# #6 Home Work - Hybrid A* Algorithm Report

Hybrid A* Algorithm는 기존 A* Algorithm과 다르게 complete와 optimal하지 않은 알고리즘이다.
하지만, Continuous와 Drivable을 중요시하기 때문에 실제로 구현하는데 있어서 A* 보다 좋을 수 있다.

이동하는 물체는 장애물이 있는 grid 상에서 회전할 수 있는 조향 각도에 대한 제한을 가지며 이동하게 하였다.

과제를 수행하기 위한 알고리즘들은 다음과 같이 구현하였다.

* `def expand(self, current, goal)`:
  
        g = current['g']
        x, y, theta = current['x'], current['y'], current['t']


        # The g value of a newly expanded cell increases by 1 from the
        # previously expanded cell.
        g2 = g + 1
        next_states = []
        # Consider a discrete selection of steering angles.
        for delta_t in range(self.omega_min, self.omega_max+1, self.omega_step):
            # print("delta_t : ", delta_t)
            omega = (self.speed / self.length) * math.tan(math.radians(delta_t))
            x2 = x + self.speed * math.cos(theta)
            y2 = y + self.speed * math.sin(theta)

            theta2 = (theta + omega) % (2*math.pi)
            f2 = g2 + self.heuristic(x2, y2, goal)
            state = {'x': x2, 'y': y2, 't': theta2, 'g': g2, 'f': f2}

            if 0 <= x2 < self.dim[1]:
                if 0 <= y2 < self.dim[2]:
                    next_states.append(state)

                    # if self.closed[(x2, y2)] == 0:
                    # if grid[(x_index, y_index)] == 0:
            # print("delta_t :", delta_t)
            # print("state :", next_states)


            # TODO: implement the trajectory generation based on
            # a simple bicycle model.
            # Let theta2 be the vehicle's heading (in radian)
            # between 0 and 2 * PI.
            # Check validity and then add to the next_states list.


        return next_states

* 위의 과정은 expand 과정으로, 현재 state에서 이동할 수 있는 next state들에 대한 값을 반환한다. 
* 고정된 속도와 길이가 주어진 조건에서 motion_model를 통해 이동할 수 있는 다음 x, y에 대한 위치와 조향각들을 계산한다. 
* 이때, 이동하는 좌표가 grid(맵)상에서 벗어났는지 확인하고, 시작점으로부터 현재까지 이동한 g cost와 현재 위치에서 목적지까지에 대한 heuristic 함수를 계산하여 f cost를 구한다.
* expand 함수로부터 반환되는 state들은 x와 y위치, 조향각, g cost와 f cost를 포함한다.


* `def search(self, grid, start, goal)`:
  
        theta = start[-1]
        stack = self.theta_to_stack_num(theta)
        g = 0
        s = {
            'f': self.heuristic(start[0], start[1], goal),
            'g': g,
            'x': start[0],
            'y': start[1],
            't': theta,
        }
        self.final = s
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]
    
        while len(opened) > 0:
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break
    
            next_states = self.expand(curr, goal)
    
            for n in next_states:
                x_index, y_index = self.idx(n['x']), self.idx(n['y'])
                dist_x = abs(self.idx(x)-x_index)
                dist_y = abs(self.idx(y)-y_index)
                dist = dist_x + dist_y
                
                # if dist > 1:
                #     continue
                
                stack = self.theta_to_stack_num(n['t'])
                if 0 <= x_index < self.dim[1]:
                    if 0 <= y_index < self.dim[2]:
                        if grid[(x_index, y_index)] == 0:
                            if self.closed[stack][x_index][y_index] == 0:
                                self.closed[stack][x_index][y_index] = 1
                                self.came_from[stack][x_index][y_index] = curr
                                opened.append(n)
                                total_closed = total_closed + 1
    
        else:
            found = False
    
        return found, total_closed

* search 함수는 open list에 state들이 없거나 혹은 목적지에 도착할 때까지 계속 반복하여 목적지까지의 경로를 찾는다. 
* 시작점으로부터 시작하여 grid 상의 cell을 이동하면서 grid 상에서 벗어났는지, 장애물이 있는 cell인지, 이미 탐색한 cell인지 체크한다. 
* 3가지 조건에 해당되는 cell이 아니라면 해당 cell을 closed list에 추가하고 시작점에서 현재 cell까지 이동했을 때 지나간 cell들의 기록들을 저장한 후 open list에 추가한다.


* `def theta_to_stack_num(self, theta)`:

        degree = math.degrees(theta)
        value = 360/self.NUM_THETA_CELLS
        stack = (degree/value)

        if stack == self.NUM_THETA_CELLS:
            stack = 0

        return int(stack)

* theta_to_stack_num 함수는 자동차의 방향에 기반하여 stack의 index를 부여하는 함수이다. 0~360도 사이에 있는 stack들 중 입력받은 각도들에 따른 stack의 index를 반환한다.

* `def heuristic(self, x, y, goal)`:

        #Euclidean Distance
        h_cost = np.sqrt((goal[1]-y)**2+(goal[0]-x)**2)
        return h_cost

* heuristic 비용의 경우, 현재 위치와 목적지까지의 Euclidean Distance를 이용하였다.

---

## Result

자동차의 speed와 NUM_THETA_CELLS을 변경하여 다음과 같은 결과를 얻어냈다.

* speed : 0.5 & NUM_THETA_CELLS : 90
![HA result1][result1]
  

* speed : 1.0 & NUM_THETA_CELLS : 90
![HA result2][result2] 
  

* speed : 0.5 & NUM_THETA_CELLS : 180
![HA result3][result3] 
   

* speed : 1.0 & NUM_THETA_CELLS : 180
![HA result4][result4] 
    

* speed : 0.5 & NUM_THETA_CELLS : 360 (교수님과 결과가 다른 것 같습니다. 제가 구현을 제대로 하지 못한 것 같습니다.)
![HA result5][result5] 
    

* speed : 1.0 & NUM_THETA_CELLS : 360
![HA result6][result6] 
  

---
[//]: # (Image References)
[has-example]: ./hybrid_a_star/has_example.png
[ptg-example]: ./PTG/ptg_example.png

## Assignment: Hybrid A* Algorithm

In directory [`./hybrid_a_star`](./hybrid_a_star), a simple test program for the hybrid A* algorithm is provided. Run the following command to test:

```
$ python main.py
```

The program consists of three modules:

* `main.py` defines the map, start configuration and end configuration. It instantiates a `HybridAStar` object and calls the search method to generate a motion plan.
* `hybrid_astar.py` implements the algorithm.
* `plot.py` provides an OpenCV-based visualization for the purpose of result monitoring.

You have to implement the following sections of code for the assignment:

* Trajectory generation: in the method `HybridAStar.expand()`, a simple one-point trajectory shall be generated based on a basic bicycle model. This is going to be used in expanding 3-D grid cells in the algorithm's search operation.
* Hybrid A* search algorithm: in the method `HybridAStar.search()`, after expanding the states reachable from the current configuration, the algorithm must process each state (i.e., determine the grid cell, check its validity, close the visited cell, and record the path. You will have to write code in the `for n in next_states:` loop.
* Discretization of heading: in the method `HybridAStar.theta_to_stack_num()`, you will write code to map the vehicle's orientation (theta) to a finite set of stack indices.
* Heuristic function: in the method `HybridAStar.heuristic()`, you define a heuristic function that will be used in determining the priority of grid cells to be expanded. For instance, the distance to the goal is a reasonable estimate of each cell's cost.

You are invited to tweak various parameters including the number of stacks (heading discretization granularity) and the vehicle's velocity. It will also be interesting to adjust the grid granularity of the map. The following figure illustrates an example output of the program with the default map given in `main.py` and `NUM_THETA_CELLS = 360` while the vehicle speed is set to 0.5.

![Example Output of the Hybrid A* Test Program][has-example]

---

## Experiment: Polynomial Trajectory Generation

In directory [`./PTG`](./PTG), a sample program is provided that tests polynomial trajectory generation. If you input the following command:

```
$ python evaluate_ptg.py
```

you will see an output such as the following figure.

![Example Output of the Polynomial Trajectory Generator][ptg-example]

Note that the above figure is an example, while the result you get will be different from run to run because of the program's random nature. The program generates a number of perturbed goal configurations, computes a jerk minimizing trajectory for each goal position, and then selects the one with the minimum cost as defined by the cost functions and their combination.

Your job in this experiment is:

1. to understand the polynomial trajectory generation by reading the code already implemented and in place; given a start configuration and a goal configuration, the algorithm computes coefficient values for a quintic polynomial that defines the jerk minimizing trajectory; and
2. to derive an appropriate set of weights applied to the cost functions; the mechanism to calculate the cost for a trajectory and selecting one with the minimum cost is the same as described in the previous (Week 6) lecture.

Experiment by tweaking the relative weight for each cost function. It will also be very interesting to define your own cost metric and implement it using the information associated with trajectories.
