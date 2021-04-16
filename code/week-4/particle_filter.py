import numpy as np
from helpers import distance
from math import pi

class ParticleFilter:
    def __init__(self, num_particles):
        self.initialized = False
        self.num_particles = num_particles

    # Set the number of particles.
    # Initialize all the particles to the initial position
    #   (based on esimates of x, y, theta and their uncertainties from GPS)
    #   and all weights to 1.0.
    # Add Gaussian noise to each particle.
    def initialize(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = []
        for i in range(self.num_particles):
            self.particles.append({
                'x': np.random.normal(x, std_x),
                'y': np.random.normal(y, std_y),
                't': np.random.normal(theta, std_theta),
                'w': 1.0,
                'assoc': [],
            })
        # print(self.particles)
        self.initialized = True

    # Add measurements to each particle and add random Gaussian noise.
    def predict(self, dt, velocity, yawrate, std_x, std_y, std_theta):
        # Be careful not to divide by zero.
        v_yr = velocity / yawrate if yawrate else 0
        yr_dt = yawrate * dt
        for p in self.particles:
            # We have to take care of very small yaw rates;
            #   apply formula for constant yaw.
            if np.fabs(yawrate) < 0.0001:
                xf = p['x'] + velocity * dt * np.cos(p['t'])
                yf = p['y'] + velocity * dt * np.sin(p['t'])
                tf = p['t']
            # Nonzero yaw rate - apply integrated formula.
            else:
                xf = p['x'] + v_yr * (np.sin(p['t'] + yr_dt) - np.sin(p['t']))
                yf = p['y'] + v_yr * (np.cos(p['t']) - np.cos(p['t'] + yr_dt))
                tf = p['t'] + yr_dt
            p['x'] = np.random.normal(xf, std_x)
            p['y'] = np.random.normal(yf, std_y)
            p['t'] = np.random.normal(tf, std_theta)

    # Find the predicted measurement that is closest to each observed
    #   measurement and assign the observed measurement to this
    #   particular landmark.
    def associate(self, predicted, observations):
        associations = []
        # For each observation, find the nearest landmark and associate it.
        #   You might want to devise and implement a more efficient algorithm.
        for o in observations:
            min_dist = -1.0
            for p in predicted:
                dist = distance(o, p)
                if min_dist < 0.0 or dist < min_dist:
                    min_dist = dist
                    min_id = p['id']
                    min_x = p['x']
                    min_y = p['y']
            association = {
                'id': min_id,
                'x': min_x,
                'y': min_y,
            }
            associations.append(association)
        # Return a list of associated landmarks that corresponds to
        #   the list of (coordinates transformed) predictions.
        return associations

    # Update the weights of each particle using a multi-variate
    #   Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
        # TODO: For each particle, do the following:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
        # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates.
        # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
        #    association algorithm.
        # 4. Calculate probability of this set of observations based on
        #    a multi-variate Gaussian distribution (two variables being
        #    the x and y positions with means from associated positions
        #    and variances from std_landmark_x and std_landmark_y).
        #    The resulting probability is the product of probabilities
        #    for all the observations.
        # 5. Update the particle's weight by the calculated probability.

        for p in self.particles:
            visible_landmarks = []
            for ID in map_landmarks:
                x = map_landmarks[ID]['x']
                y = map_landmarks[ID]['y']
                if distance(p, map_landmarks[ID]) < sensor_range :
                    visible_landmarks.append({'id': ID, 'x': x, 'y': y})

            # print("visible_landmarks :", visible_landmarks)
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
            # print("observations_in_Map_frame :", observations_in_Map_frame)
            # 2. Transform each observed landmark 's coordinates from the
            #    particle's coordinate system to the map's coordinates.

            landmarks_associated = self.associate(visible_landmarks, observations_in_Map_frame)
            # print("landmarks_associated :", landmarks_associated)

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

                p['assoc'].append(landmarks_associated[i]['id'])
            # 5. Update the particle's weight by the calculated probability.



    # Resample particles with replacement with probability proportional to
    #   their weights.

    def resample(self):
        # TODO: Select (possibly with duplicates) the set of particles
        #       that captures the posteior belief distribution, by
        # 1. Drawing particle samples according to their weights.
        # 2. Make a copy of the particle; otherwise the duplicate particles
        #    will not behave independently from each other - they are
        #    references to mutable objects in Python.
        # Finally, self.particles shall contain the newly drawn set of
        #   particles.
        import copy

        resample_particle = []
        weights = np.array([p['w'] for p in self.particles])
        weights_sum = np.sum(weights)
        weights_normal = weights / weights_sum
        # print("weights:", weights)
        # print("weights_sum:", weights_sum)
        # print("weights_normal:",weights_normal)
        r = np.random.random_sample()/self.num_particles
        # print("r:",r)
        c = weights_normal[0]
        i = 0
        # print(range(1,self.num_particles))
        for m in range(1,self.num_particles) :
            # print("m:", m)
            U = r + (m-1)/self.num_particles
            # print("U:",U)
            while U > c :
                i = i + 1
                # print("i:", i)
                c = c + weights_normal[i]
                # print("c:",c)

            resample_particle.append(copy.deepcopy(self.particles[i]))

        self.particles = resample_particle


    # Choose the particle with the highest weight (probability)
    def get_best_particle(self):
        highest_weight = -1.0
        for p in self.particles:
            if p['w'] > highest_weight:
                highest_weight = p['w']
                best_particle = p

        return best_particle

    def multivariate_normal_pdf(self, x, m, x_s, y_s):
        cov = np.array([
            [x_s ** 2, 0],
            [0, y_s ** 2]
        ])
        inv_cov = np.linalg.inv(cov)
        det_cov = np.linalg.det(cov)
        x_m = x-m
        one_over_sqrt_2pi = 1 / (np.sqrt((2 * pi) ** 2 * det_cov))

        return one_over_sqrt_2pi * np.exp(-0.5 * np.dot(np.dot(np.transpose(x_m), inv_cov), x_m))