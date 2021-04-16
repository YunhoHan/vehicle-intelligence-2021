# Week 4 - Motion Model & Particle Filters

---

[//]: # (Image References)
[empty-update]: ./empty-update.gif
[example]: ./example.gif
[plot]: ./Particle_filter_of_yunho.gif

# #3 Home Work - Particle Filter Report

The function of update_weights for Particle Filter is programmed as follows:

* `update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks)`:

        for p in self.particles:
            visible_landmarks = []
            for ID in map_landmarks:
                x = map_landmarks[ID]['x']
                y = map_landmarks[ID]['y']
                if distance(p, map_landmarks[ID]) < sensor_range :
                    visible_landmarks.append({'id': ID, 'x': x, 'y': y})
  
            if not visible_landmarks:
                continue
            # 1. Select the set of landmarks that are visible
            #    within the sensor range.

            observations_in_Map_frame = []
            for observation_in_Car_frame in observations:
                x = p['x'] + observation_in_Car_frame['x']*np.cos(p['t']) - observation_in_Car_frame['y'] * np.sin(p['t'])
                y = p['y'] + observation_in_Car_frame['x']*np.sin(p['t']) + observation_in_Car_frame['y'] * np.cos(p['t'])
                observation_in_Map_frame = {
                    'x': x,
                    'y': y
                }
                observations_in_Map_frame.append(observation_in_Map_frame)
            # 2. Transform each observed landmark 's coordinates from the
            #    particle's coordinate system to the map's coordinates.

            landmarks_associated = self.associate(visible_landmarks, observations_in_Map_frame)
            # 3. Associate each transformed observation to one of the
            #    predicted (selected in Step 1) landmark positions.
            #    Use self.associate() for this purpose - it receives
            #    the predicted landmarks and observations; and returns
            #    the list of landmarks by implementing the nearest-neighbour
            #    association algorithm.

            p['w'] = 1.0
            p['assoc'] = []
            for i in range(len(landmarks_associated)):
                x = np.array([
                    [observations_in_Map_frame[i]['x']],
                    [observations_in_Map_frame[i]['y']]
                ])
                m = np.array([
                    [landmarks_associated[i]['x']],
                    [landmarks_associated[i]['y']]
                ])
                p['w'] = p['w']*self.multivariate_normal_pdf(x,m,std_landmark_x, std_landmark_y)
            # 4. Calculate probability of this set of observations based on
            #    a multi-variate Gaussian distribution (two variables being
            #    the x and y positions with means from associated positions
            #    and variances from std_landmark_x and std_landmark_y).
            #    The resulting probability is the product of probabilities
            #    for all the observations.
  
            #    multivariate_normal_pdf is to get multivariate normal distributions.

                p['assoc'].append(landmarks_associated[i]['id'])
            # 5. Update the particle's weight by the calculated probability.

The function for multivariate normal distributions is programmed as follows:

* `multivariate_normal_pdf(self, x, m, x_s, y_s)`:

        cov = np.array([
            [x_s ** 2, 0],
            [0, y_s ** 2]
        ])
        inv_cov = np.linalg.inv(cov)
        det_cov = np.linalg.det(cov)
        x_m = x-m
        one_over_sqrt_2pi = 1 / (np.sqrt((2 * pi) ** 2 * det_cov))

        return one_over_sqrt_2pi * np.exp(-0.5 * np.dot(np.dot(np.transpose(x_m), inv_cov), x_m))

The function of resampling for Particle Filter is programmed as follows:

* `resample(self)`:

        import copy

        resample_particle = []
        weights = np.array([p['w'] for p in self.particles])
        weights_sum = np.sum(weights)
        weights_normal = weights / weights_sum
        # 1. Normalize weight of particles

        r = np.random.random_sample()/self.num_particles
        c = weights_normal[0]
        i = 0
  
        for m in range(1,self.num_particles) :

            U = r + (m-1)/self.num_particles
            while U > c :
                i = i + 1
                c = c + weights_normal[i]

            resample_particle.append(copy.deepcopy(self.particles[i]))
            # 2. 결과적으로 가중치가 큰 녀석이 중복적으로 들어가게 된다.
  
        self.particles = resample_particle


![Expected Result of Particle Filter][plot]
The above graph is homework result.

## Assignment

You will complete the implementation of a simple particle filter by writing the following two methods of `class ParticleFilter` defined in `particle_filter.py`:

* `update_weights()`: For each particle in the sample set, calculate the probability of the set of observations based on a multi-variate Gaussian distribution.
* `resample()`: Reconstruct the set of particles that capture the posterior belief distribution by drawing samples according to the weights.

To run the program (which generates a 2D plot), execute the following command:

```
$ python run.py
```

Without any modification to the code, you will see a resulting plot like the one below:

![Particle Filter without Proper Update & Resample][empty-update]

while a reasonable implementation of the above mentioned methods (assignments) will give you something like

![Particle Filter Example][example]

Carefully read comments in the two method bodies and write Python code that does the job.
