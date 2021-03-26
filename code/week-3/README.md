# Week 3 - Kalman Filters, EKF and Sensor Fusion

---

[//]: # (Image References)
[yunho-result]: ./EKF/the_EKF_Result_of_yunho.png

# #2 Home Work - Extended Kalman Filter Report

The function for Extended Kalman Filter is programmed as follows:

* `def update_ekf(self, z)`:

        px, py, vx, vy = self.x
        Hj = Jacobian(self.x)  # 3X4 matrix

        S = np.dot(np.dot(Hj, self.P), Hj.T) + self.R #3X3 matrix
        K = np.dot(np.dot(self.P, Hj.T), np.linalg.inv(S)) #4X3 matrix

        c1 = px * px + py * py
        c2 = sqrt(c1)
        hx = [c2, atan2(py,px), (px * vx + py * vy)/c2]

        PI = 3.14159;
        y = z - hx

        if (y[1] > PI) :
            y[1] = (y[1]-2*PI)
        if (y[1] < -PI) :
            y[1] = (y[1]+2*PI)

        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, Hj), self.P)

        # TODO: Implement EKF update for radar measurements
        # 1. Compute Jacobian Matrix Hj
        # 2. Calculate S = Hj * P' * Hj^T + R
        # 3. Calculate Kalman gain K = P' * Hj^T * inv(S)
        # 4. Estimate y = z - h(x')
        # 5. Normalize phi so that it is between -PI and +PI
        # 6. Calculate new estimates
        #    x = x' + K * y
        #    P = (I - K * Hj) * P'

![Testing of EKF with Sensor Fusion][yunho-result]
The above graph is homework result.

---
[kalman-result]: ./kalman_filter/graph.png
[EKF-results]: ./EKF/plot.png
## Kalman Filter Example

In directory [`./kalman_filter`](./kalman_filter), a sample program for a small-scale demonstration of a Kalman filter is provided. Run the following command to test:

```
$ python testKalman.py
```

This program consists of four modules:

* `testKalman.py` is the module you want to run; it initializes a simple Kalman filter and estimates the position and velocity of an object that is assumed to move at a constant speed (but with measurement error).
* `kalman.py` implements a basic Kalman fitler as described in class.
* `plot.py` generates a plot in the format shown below.
* `data.py` provides measurement and ground truth data used in the example.

The result of running this program with test input data is illustrated below:

![Testing of Kalman Filter Example][kalman-result]

Interpretation of the above results is given in the lecture.

In addition, you can run `inputgen.py` to generate your own sample data. It will be interesting to experiment with a number of data sets with different characteristics (mainly in terms of variance, i.e., noise, involved in control and measurement).

---

## Assignment - EFK & Sensor Fusion Example

In directory [`./EKF`](./EKF), template code is provided for a simple implementation of EKF (extended Kalman filter) with sensor fusion. Run the following command to test:

```
$ python run.py
```

The program consists of five modules:

* `run.py` is the modele you want to run. It reads the input data from a text file ([data.txt](./EKF/data.txt)) and feed them to the filter; after execution summarizes the result using a 2D plot.
* `sensor_fusion.py` processees measurements by (1) adjusting the state transition matrix according to the time elapsed since the last measuremenet, and (2) setting up the process noise covariance matrix for prediction; selectively calls updated based on the measurement type (lidar or radar).
* `kalman_filter.py` implements prediction and update algorithm for EKF. All the other parts are already written, while completing `update_ekf()` is left for assignment. See below.
* `tools.py` provides a function `Jacobian()` to calculate the Jacobian matrix needed in our filter's update algorithm.
*  `plot.py` creates a 2D plot comparing the ground truth against our estimation. The following figure illustrates an example:

![Testing of EKF with Sensor Fusion][EKF-results]

### Assignment

Complete the implementation of EKF with sensor fusion by writing the function `update_ekf()` in the module `kalman_filter`. Details are given in class and instructions are included in comments.
