# Week 6 - Prediction & Behaviour Planning

---
[//]: # (Image References)
[result1]: ./GNB_result.png
# #5-1 Home Work - Gaussian Naive Bayes Report

Gaussian Naive Bayes 과제는 수집된 데이터를 이용하여 각 주행(왼쪽차선 이동, 직진 이동, 오른쪽차선 이동)의 평균들과 표준편차들을 계산한다. 
이후, Observation으로 취득된 데이터들과 평균들 및 표준편차들을 이용해 가능성이 높은 주행 경로를 예측하는 것이다. 
구현 알고리즘은 다음과 같이 구현하였다.

* `def train(self, X, Y)`:
  
        state_arr = []
        label_arr = []

        left_s, left_d, left_sdot, left_ddot = [], [], [], []
        keep_s, keep_d, keep_sdot, keep_ddot = [], [], [], []
        right_s, right_d, right_sdot, right_ddot = [], [], [], []

        # print("X : ", X)
        for x_component in X:
            [s, d, s_dot, d_dot] = self.process_vars(x_component)
            var = [s, d, s_dot, d_dot]
            state_arr.append(var)

        for y_component in Y:
            label = y_component
            label_arr.append(label)

        for label_arr, state_arr in list(zip(label_arr, state_arr)):
            if label_arr == "left":
                left_s.append(state_arr[0])
                left_d.append(state_arr[1])
                left_sdot.append(state_arr[2])
                left_ddot.append(state_arr[3])
            elif label_arr == "keep":
                keep_s.append(state_arr[0])
                keep_d.append(state_arr[1])
                keep_sdot.append(state_arr[2])
                keep_ddot.append(state_arr[3])
            elif label_arr == "right":
                right_s.append(state_arr[0])
                right_d.append(state_arr[1])
                right_sdot.append(state_arr[2])
                right_ddot.append(state_arr[3])

        self.left_mean = [np.mean(left_s), np.mean(left_d), np.mean(left_sdot), np.mean(left_ddot)]
        self.left_std = [np.std(left_s), np.std(left_d), np.std(left_sdot), np.std(left_ddot)]

        self.keep_mean = [np.mean(keep_s), np.mean(keep_d), np.mean(keep_sdot), np.mean(keep_ddot)]
        self.keep_std = [np.std(keep_s), np.std(keep_d), np.std(keep_sdot), np.std(keep_ddot)]

        self.right_mean = [np.mean(right_s), np.mean(right_d), np.mean(right_sdot), np.mean(right_ddot)]
        self.right_std = [np.std(right_s), np.std(right_d), np.std(right_sdot), np.std(right_ddot)]

        # Given an observation (s, s_dot, d, d_dot), predict which behaviour
        # the vehicle is going to take using GNB.

* 위의 과정은 train 혹은 학습과정으로 데이터에 포함된 라벨에 따라 분류하여 각각의 주행에서 s, d, sdot, ddot에 대한 평균과 표준편차들을 계산하였다.
  

* `def predict(self, observation)`:
  
        index = 0
        left_prob, keep_prob, right_prob = 1.0, 1.0, 1.0
        left_normalize, keep_normalize, right_normalize = 0.0, 0.0, 0.0
        for data in observation:
            left_prob *= gaussian_prob(data, self.left_mean[index], self.left_std[index])
            keep_prob *= gaussian_prob(data, self.keep_mean[index], self.keep_std[index])
            right_prob *= gaussian_prob(data, self.right_mean[index], self.right_std[index])

            left_normalize += gaussian_prob(data, self.left_mean[index], self.left_std[index])
            keep_normalize += gaussian_prob(data, self.keep_mean[index], self.keep_std[index])
            right_normalize += gaussian_prob(data, self.right_mean[index], self.right_std[index])

            index += 1

        left_normalize = 1 / left_normalize
        keep_normalize = 1 / keep_normalize
        right_normalize = 1 / right_normalize

        left_prob = left_prob * left_normalize
        keep_prob = keep_prob * keep_normalize
        right_prob = right_prob * right_normalize

        prob_label = np.argmax([left_prob, keep_prob, right_prob])

        return self.classes[prob_label]
  
* train에서 계산한 각 주행에 대한 평균과 표준편차 및 Observation data의 데이터들을 이용하여 Observation에서 가장 높은 확률을 가진 주행 경로를 예측하였다.

![GNB result][result1]

* 위의 그림은 경로를 예측하는데 있어서 계산된 정확도를 나타낸다. 최종 정확도는 84.40 %의 결과를 나타내었다.

---
[result2]: ./BP_result.png
# #5-2 Home Work - Behaviour Planning Report

cost(각 차선에서의 속도, goal lane과의 거리)를 고려하여 현재 자동차에 대한 Trajectory을 구하는 Behaviour Planning을 수행하였다.
구현 알고리즘은 다음과 같이 작성하였다.

* `def choose_next_state(self, predictions)`:
  
        index = 0
        left_prob, keep_prob, right_prob = 1.0, 1.0, 1.0
        left_normalize, keep_normalize, right_normalize = 0.0, 0.0, 0.0
        for data in observation:
            left_prob *= gaussian_prob(data, self.left_mean[index], self.left_std[index])
            keep_prob *= gaussian_prob(data, self.keep_mean[index], self.keep_std[index])
            right_prob *= gaussian_prob(data, self.right_mean[index], self.right_std[index])

            left_normalize += gaussian_prob(data, self.left_mean[index], self.left_std[index])
            keep_normalize += gaussian_prob(data, self.keep_mean[index], self.keep_std[index])
            right_normalize += gaussian_prob(data, self.right_mean[index], self.right_std[index])

            index += 1

        left_normalize = 1 / left_normalize
        keep_normalize = 1 / keep_normalize
        right_normalize = 1 / right_normalize

        left_prob = left_prob * left_normalize
        keep_prob = keep_prob * keep_normalize
        right_prob = right_prob * right_normalize

        prob_label = np.argmax([left_prob, keep_prob, right_prob])

        return self.classes[prob_label]

* 위의 과정은 현재 자동차의 위치에서 다음 state(KL, PLCL, PLCR, LCL, LCR), 즉 어떤 Behaviour을 통해 어디로 갈 것인지에 대해 결정한다. 
* 예상되는 state들에 대해서 수행이 가능한 state들인지 체크한 후, 현재 state에서 갈 수 있는 state로 이동하는 궤적을 계산한다.
* 이후, 각 궤적에 대한 cost를 계산하여 가장 비용이 최소가 되는 궤적을 최종적으로 선택한다.


* `def goal_distance_cost(vehicle, trajectory, predictions, data)`:

        goal_lane = vehicle.goal_lane
        current_dist = vehicle.s
    
        weight1 = 5.0
        weight2 = 2.0
        weight3 = 1.0
    
        cost = weight1 * (current_dist / (current_dist + data.end_distance_to_goal)) + weight2 * abs(goal_lane - data.intended_lane) + weight3 * abs(goal_lane - data.final_lane)
    
        return cost
  
* 위의 과정은 goal_lane과 현재 자동차가 주행중인 lane에 대한 cost를 계산한다.
* 총 3개의 가중치를 이용하여 비용을 계산하였다.
* 가중치1(weight1)은 목표 지점에 가까울수록 가중치가 증가한다. 
  그리고, 목적지에 가까워지고 있는데 현재 주행하는 차선이 목표지점에 멀어질수록, cost는 더 크게 증가한다.
* 가중치2(weight2)에 대한 부분은 목표 lane과 intended_lane 사이에 대한 거리 비용이다. 
* 가중치3(weight3)에 대한 부분은 목표 lane과 final lane에 대한 거리 비용을 나타낸다.


* `def inefficiency_cost(vehicle, trajectory, predictions, data)`:

      current_dist = vehicle.s
    
      lane_num = vehicle.lanes_available
      lane = []
      lane_weights = []
      num = 0
    
      intended_weight, final_weight = 0, 0
    
      for index in range(lane_num):
          lane.append(num)
          num = num + 1
    
      for index in range(lane_num):
          weight = 2.0*(lane_num-index)
          lane_weights.append(weight)
    
      for i in range(lane_num):
          if data.intended_lane == lane[i]:
              intended_weight = lane_weights[i]
    
          if data.final_lane == lane[i]:
              final_weight = lane_weights[i]
    
      cost = (current_dist / (current_dist + data.end_distance_to_goal)) + (intended_weight + final_weight)
      if current_dist > data.end_distance_to_goal:
          cost = 0.2*cost
    
      return cost

* 위의 과정은 각 lane에 대해서 일정한 속도로 주행해야한다는 조건이 있다. 
* cost는 각 lane에 해당하는 속도에 따라 다르며, 속도가 느릴수록 시간이 더 많이 소요되기에 판단되어 cost를 더 증가시키게 하였다.
* 가장 빠른 차선을 통해 빠른 속도로 주행한 뒤, 목적지에 가까워 졌을 때 목표 lane쪽으로 이동하는 것이 최소의 비용을 가진 궤적을 찾는다고 생각된다. 
  따라서, inefficient_cost는 각 차선[1번 lane, 2번 lane, 3번 lane, 4번 lane]에 대해 [8 6 4 2] 로 cost를 설정하였다. 
  즉, 1번 lane은 가장 느린 속도를 가지고 있으므로 가장 큰 cost를 지닌다.
* 현재 거리가 목적지로부터 남은 거리보다 클 때, inefficient_cost보다 goal_distance_cost에 대한 비중을 더 높여서 목표 lane쪽으로 이동하도록 하였다.


![BP result][result2]

* 위의 그림은 시뮬레이션 결과이다. Vehicle이 목표지점에 도달하기까지 33초가 소요되었다.

---
## Assignment #1

Under the directory [./GNB](./GNB), you are given two Python modules:

* `prediction.py`: the main module you run. The `main()` function does two things: (1) read an input file ([`train.json`](./GNB/train.json)) and train the GNB (Gaussian Naive Bayes) classifier using the data stored in it, and (2) read another input file ([`test.json`](./GNB/test.json)) and make predictions for a number of data points. The accuracy measure is taken and displayed.
* `classifier.py`: main implementation of the GNB classifier. You shall implement two methods (`train()` and `precict()`), which are used to train the classifier and make predictions, respectively.

Both input files ([`train.json`](./GNB/train.json) and [`test.json`](./GNB/test.json)) have the same format, which is a JSON-encoded representation of training data set and test data set, respectively. The format is shown below:

```
{
	"states": [[s_1, d_1, s_dot_1, d_dot_1],
	           [s_2, d_2, s_dot_2, d_dot_2],
	           ...
	           [s_n, d_n, s_dot_n, d_dot_n]
	          ],
	"labels": [L_1, L_2, ..., L_n]
}
```

The array `"states"` have a total of `n` items, each of which gives a (hypothetically) measured state of a vehicle, where `s_i` and `d_i` denote its position in the Frenet coordinate system. In addition, `s_dot_i` and `d_dot_i` give their first derivates, respectively. For each measured state, a label is associated (given in the `"labels"` array) that represents the vehicle's behaviour. The label is one of `"keep"`, `"left"`, and `"right"`, which denote keeping the current lane, making a left turn, and making a right turn, respectively.

The training set has a total of 750 data points, whereas the test set contains 250 data points with the ground truth contained in `"labels"`.

The GNB classifier is trained by computing the mean and variance of each component in the state variable for each observed behaviour. Later it is used to predict the behaviour by computing the Gaussian probability of an observed state for each behaviour and taking the maximum. You are going to implement that functionality. For convcenience, a separate function `gaussian_prob()` is already given in the module `classifier.py`.


---

## Assignment #2

Under the directory [./BP](./BP), you are given four Python modules:

* `simulate_behavior.py`: the main module you run. It instantiates a simple text-based simulation environment and runs it using the configuration specified in the same module.
* `road.py`: `class Road` is implemented here. It captures the state of the simulated road with a number of vehicles (including the ego) running on it, and visualizes it using terminal output.
* `vehicle.py`: `class Vehicle` implements the states of a vehicle and its transition, along with the vehicle's dynamics based on a simple kinematic assumption. Note that a vehicle's trajectory is represented by two instances of object of this class, where the first one gives the current state and the second one predicts the state that the vehicle is going to be in after one timestep.
* `cost_functions.py`: implementation of cost functions governing the state transition of the ego vehicle. The main job required for your assignment is to provide an adequate combination of cost functions by implementing them in this module.

### Task 1

Implement the method `choose_next_state()` in `vehicle.py`. It should

* determine which state transitions are possible from the current state (`successor_states()` function in the same module will be helpful),
* calculate cost for each state transition using the trajectory generated for each behaviour, and
* select the minimum cost trajectory and return it.

Note that you must return a planned trajectory (as described above) instead of the state that the vehicle is going to be in.

### Task 2

In `cost_functions.py`, templates for two different cost functions (`goal_distance_cost()` and `inefficiency_cost()`) are given. They are intended to capture the cost of the trajectory in terms of

* the lateral distance of the vehicle's lane selection from the goal position, and
* the time expected to be taken to reach the goal (because of different lane speeds),

respectively.

Note that the range of cost functions should be carefully defined so that they can be combined by a weighted sum, which is done in the function `calculate_cost()` (to be used in `choose_next_state()` as described above). In computing the weighted sum, a set of weights are used. For example, `REACH_GOAL` and `EFFICIENCY` are already defined (but initialized to zero values). You are going to find out a good combination of weights by an empirical manner.

You are highly encouraged to experiment with your own additional cost functions. In implementing cost functions, a trajectory's summary (defined in `TrajectoryData` and given by `get_helper_data()`) can be useful.

You are also invited to experiment with a number of different simulation settings, especially in terms of

* number of lanes
* lane speed settings (all non-ego vehicles follow these)
* traffic density (governing the number of non-ego vehicles)

and so on.

Remember that our state machine should be geared towards reaching the goal in an *efficient* manner. Try to compare a behaviour that switches to the goal lane as soon as possible (note that the goal position is in the slowest lane in the given setting) and one that follows a faster lane and move to the goal lane as the remaining distance decreases. Observe different behaviour taken by the ego vehicle when different weights are given to different cost functions, and also when other cost metrics (additional cost functions) are used.
