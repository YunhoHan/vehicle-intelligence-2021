# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./Markov_of_yunho.gif

# #1 Home Work - Markov Localization Report

* `motion_model()`:


    def motion_model(position, mov, priors, map_size, stdev):

        # Initialize the position's probability to zero.

        position_prob = 0.0

        for p in range(map_size) :
            position_prob = position_prob + norm_pdf(position - p, mov, stdev) * priors[p]
        # The prediction of motion model probability can be calculated by using the above code.
        # The code is sum of multiplying prior position by normal distribution.

        # TODO: Loop over state space for all possible prior positions,
        # calculate the probability (using norm_pdf) of the vehicle
        # moving to the current position from that prior.
        # Multiply this probability to the prior probability of
        # the vehicle "was" at that prior position.
        return position_prob

* `observation_model()`:


    def observation_model(landmarks, observations, pseudo_ranges, stdev):
        
        # Initialize the measurement's probability to one.
        distance_prob = 1.0

        if len(observations) == 0 or len(observations) > len(pseudo_ranges):
            distance_prob = 0.0

        else :
            for i in range(len(observations)):
                distance_prob = distance_prob * norm_pdf(observations[i], pseudo_ranges[i], stdev)

        # The observation model probability can be calculated by using the above code.
    
        # TODO: Calculate the observation model probability as follows:
        # (1) If we have no observations, we do not have any probability.
        # (2) Having more observations than the pseudo range indicates that
        #     this observation is not possible at all.
        # (3) Otherwise, the probability of this "observation" is the product of
        #     probability of observing each landmark at that distance, where
        #     that probability follows N(d, mu, sig) with
        #     d: observation distance
        #     mu: expected mean distance, given by pseudo_ranges
        #     sig: squared standard deviation of measurement
        return distance_prob

![Expected Result of Markov Localization][plot]
The above graph is homework result.
---
[plot2]: ./markov.gif

## Assignment

You will complete the implementation of a simple Markov localizer by writing the following two functions in `markov_localizer.py`:

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.

The algorithm is presented and explained in class.

All the other source files (`main.py` and `helper.py`) should be left as they are.

If you correctly implement the above functions, you expect to see a plot similar to the following:

![Expected Result of Markov Localization][plot2]

If you run the program (`main.py`) without any modification to the code, it will generate only the frame of the above plot because all probabilities returned by `motion_model()` are zero by default.
