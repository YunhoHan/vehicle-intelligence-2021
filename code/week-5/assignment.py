import numpy as np
import itertools

# Given map
grid = np.array([
    [1, 1, 1, 0, 0, 0],
    [1, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 1, 1],
    [1, 1, 1, 0, 1, 1]
])

# List of possible actions defined in terms of changes in
# the coordinates (y, x)
forward = [
    (-1,  0),   # Up
    ( 0, -1),   # Left
    ( 1,  0),   # Down
    ( 0,  1),   # Right
]

# Three actions are defined:
# - right turn & move forward
# - straight forward
# - left turn & move forward
# Note that each action transforms the orientation along the
# forward array defined above.
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

init = (4, 3, 0)    # Representing (y, x, o), where
                    # o denotes the orientation as follows:
                    # 0: up
                    # 1: left
                    # 2: down
                    # 3: right
                    # Note that this order corresponds to forward above.
goal = (2, 0)
cost = (2, 1, 20)   # Cost for each action (right, straight, left)

# EXAMPLE OUTPUT:
# calling optimum_policy_2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]

def optimum_policy_2D(grid, init, goal, cost):
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

print(optimum_policy_2D(grid, init, goal, cost))
