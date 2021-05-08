# Week 5 - Path Planning & the A* Algorithm

---
[//]: # (Image References)
[result1]: cost_2_1_2.PNG
[result2]: cost_2_1_20.PNG

# #4 Home Work - Dynamic Programming Approach Report

Dynamic Programming Approach 방법을 이용하여 경로 생성 알고리즘을 다음과 같이 구현하였다.

* `def optimum_policy_2D(grid, init, goal, cost)`:
	
		# Initialize the value function with (infeasibly) high costs.
		value = np.full((4, ) + grid.shape, 999, dtype=np.int32)
		# Initialize the policy function with negative (unused) values.
		policy = np.full((4,) + grid.shape, -1, dtype=np.int32)
		# Final path policy will be in 2D, instead of 3D.
		policy2D = np.full(grid.shape, ' ')
	
		# Apply dynamic programming with the flag change.
		change = True
		while change:
			change = False
			# This will provide a useful iterator for the state space.
			p = itertools.product(
				range(grid.shape[0]),
				range(grid.shape[1]),
				range(len(forward))
			)
			# Compute the value function for each state and
			# update policy function accordingly.
			for y, x, t in p:
				# Mark the final state with a special value that we will
				# use in generating the final path policy.
				if (y, x) == goal and value[(t, y, x)] > 0:
					value[(t, y, x)] = 0
					policy2D[(y, x)] = 'G'
					# Dynamic Programming Approach는 목적지로부터 경로를 탐색 및 cost 계산하는 알고리즘
					# y, x가 목적지일 때, cost는 0, policy2D[(y, x)]는 현재 'G'(=goal)
					change = True
					pass
				# Try to use simple arithmetic to capture state transitions.
				elif grid[(y, x)] == 0: #현재 해당 셀이 장애물이 있는지 없는지 체크
					for act, act_name in zip(action, action_name):
						direction = (t + act) % len(forward)
						# t : 방향,  act : 액션, (t+act)% len(forward)는 액션 후의 방향, 순환구조 시스템 때문에
						# ex 3+1= 4(=0)
						# ex Right + Left turn은 up -> up은 0
	
						y2, x2 = y + forward[direction][0], x + forward[direction][1]
						#move forward 에 대한 code
						if 0 <= y2 < grid.shape[0] \
							and 0 <= x2 < grid.shape[1] \
							and grid[(y2, x2)] == 0: #이동한 후의 셀이 장애물이 있는지 없는지 체크
	
							if act == action[0]:
								cost_value = cost[0]
							elif act == action[1]:
								cost_value = cost[1]
							elif act == action[2]:
								cost_value = cost[2]
	
							v2 = value[(direction, y2, x2)] + cost_value
	
							if v2 < value[(t, y, x)]:
								change = True
								value[(t, y, x)] = v2
								policy[(t, y, x)] = act
					pass
	
		# Now navigate through the policy table to generate a
		# sequence of actions to take to follow the optimal path.
	
		y, x, dir = init
		start = policy[(dir, y, x)]
		# 각 4개의 방향 층에 대한 policy에서 y,x,dir 정보에 해당하는 액션을 start로 넣는다.
	
		for i in action:
			if start == action[i]:
				start_name = action_name[i]
				policy2D[(y, x)] = start_name
				# 해당하는 스타트 액션과 액션 정보가 맞다면, 액션 이름 없데이트
	
		while policy2D[(y, x)] != 'G':
	
			# for t in range(len(forward)):
			#     if t == 0:
			#         min_value = value[t, y, x]
			#
			#     elif value[t, y, x] < min_value:
			#         min_value = value[t, y, x]
	
			for i in action:
				if (policy[(dir, y, x)] == action[i]):
					policy2D[(y, x)] = action_name[i]
					move_dir = (dir + action[i]) % len(forward)
	
			# for i in action:
			#     if policy[(dir, y, x)] == action[i]:
			#         move_dir = (dir + action[i]) % len(forward)
	
			move_y, move_x = y + forward[move_dir][0], x + forward[move_dir][1]
	
			y = move_y
			x = move_x
			dir = move_dir
	
		# Return the optimum policy generated above.
		return policy2D

* 알고리즘 초기에 방향 4가지(Up,Left,Down,Right)에 따라 4개의 그리드(맵을) 준비하고, 각 셀을 초기화하는 과정을 진행한다. 각 셀에 Cost=999, Policy=-1.  
* 위치 y,x 그리고 방향 t(Up, Left, Down, Right)에서 선택한 위치 y,x가 최종 목적지의 위치인지 확인하고, 장애물 여부를 확인하여 search를 시작한다.
* act는 Turn_Right, Straight, Turn_Left의 action 정보를 나타내며 direction = (t + d) % len(forward) 를 통해 현재 방향 상태(Up, Left, Down, Right)에서 action을 후의 방향의 정보를 계산한다.
* 이후, 이동하려는 cell에 장애물이 여부와 주어진 grid(맵)로부터 벗어났는지 체크하고, action에 따라 다른 cost 값을 부여하여 계산한다. 만일 계산한 cost 값이 이전에 저장된 값보다 작다면 action과 cost를 업데이트한다.
* 방향 문자열이 표시된 Map인 policy2D에서 초기 start지점에 action_name을 갱신한다. policy(dir, y, x)에 저장된 action 정보에 따라 policy2D에서 해당 action name을 저장한다. 이 작업은 Goal 지점을 찾을 때까지 반복하여 수행한다.

![cost1][result1]

* 위의 그림은 act (right, straight, left)의 cost가 (2,1,20)일 때, Dynamic Programming Approach 결과이다.

![cost2][result2]

* 위의 그림은 act (right, straight, left)의 cost가 (2,1,2)일 때, Dynamic Programming Approach 결과이다.

---
## Examples

We have four small working examples for demonstration of basic path planning algorithms:

* `search.py`: simple shortest path search algorithm based on BFS (breadth first search) - only calculating the cost.
* `path.py`: built on top of the above, generating an optimum (in terms of action steps) path plan.
* `astar.py`: basic implementation of the A* algorithm that employs a heuristic function in expanding the search.
* `policy.py`: computation of the shortest path based on a dynamic programming technique.

These sample source can be run as they are. Explanation and test results are given in the lecture notes.

## Assignment

You will complete the implementation of a simple path planning algorithm based on the dynamic programming technique demonstrated in `policy.py`. A template code is given by `assignment.py`.

The assignmemt extends `policy.py` in two aspects:

* State space: since we now consider not only the position of the vehicle but also its orientation, the state is now represented in 3D instead of 2D.
* Cost function: we define different cost for different actions. They are:
	- Turn right and move forward
	- Move forward
	- Turn left and move forward

This example is intended to illustrate the algorithm's capability of generating an alternative (detouring) path when left turns are penalized by a higher cost than the other two possible actions. When run with the settings defined in `assignment.py` without modification, a successful implementation shall generate the following output,

```
[[' ', ' ', ' ', 'R', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', '#'],
 ['*', '#', '#', '#', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', ' '],
 [' ', ' ', ' ', '#', ' ', ' ']]
```

because of the prohibitively high cost associated with a left turn.

You are highly encouraged to experiment with different (more complex) maps and different cost settings for each type of action.
