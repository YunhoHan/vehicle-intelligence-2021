import matplotlib.pyplot as plt
import matplotlib.animation as animation
from helper import GraphAnimator

from markov_localizer import initialize_priors
from markov_localizer import estimate_pseudo_range
from markov_localizer import motion_model
from markov_localizer import observation_model
from markov_localizer import normalize_distribution

if __name__ == '__main__':
    # Initialize graph data to an empty list
    # 그래프 초기화
    graph = []
    # Std dev for initial position
    # 초기 위치에 대한 표준 편차 초기화
    position_stdev = 1.0
    # Std dev for control (movement)
    # 초기 Control 표준 편차 초기화
    control_stdev = 1.0
    # Std dev for observation (measurement)
    # 초기 관측, 측정 표준 편차 초기화
    observation_stdev = 1.0
    # Assumed constant velocity of vehicle
    # 차량의 등속 셋팅 및 가정
    mov_per_timestep = 1.0

    # Size of the map
    # 맵의 사이즈
    map_size = 25
    # Map (landmark positions)
    # 랜드마크인 나무들의 위치
    landmark_positions = [3, 9, 14, 23]

    # Observation data
    # 관측했던 데이터??
    observations = [
        [1, 7, 12, 21],
        [0, 6, 11, 20],
        [5, 10, 19],
        [4, 9, 18],
        [3, 8, 17],
        [2, 7, 16],
        [1, 6, 15],
        [0, 5, 14],
        [4, 13],
        [3, 12],
        [2, 11],
        [1, 10],
        [0, 9],
        [8],
        [7],
        [6],
        [5],
        [4],
        [3],
        [2],
        [1],
        [0],
        [],
        [],
        [],
    ]

    # Initialize priors (initial belief)
    # 초기화 priors
    priors = initialize_priors(
        map_size, landmark_positions, position_stdev
    )
    # print(priors)

    # Cycle through timesteps
    for t in range(len(observations)):

        # print("---------------TIME STEP---------------")
        # print("t = %d" % t)
        # print(range(map_size))
        # print("-----Motion----------OBS----------------PRODUCT--")

        posteriors = [0.0] * map_size
        # Step through each pseudo position p (to determine pdf)
        for pseudo_position in range(map_size):
            # Prediction:
            # Calculate probability of the vehicle being at position p
            # 차량이 p 위치에있을 확률 계산
            motion_prob = motion_model(
                pseudo_position, mov_per_timestep, priors,
                map_size, control_stdev
            )
            # Get pseudo range
            pseudo_ranges = estimate_pseudo_range(
                landmark_positions, pseudo_position
            )
            # print("pseudo_position = %d" % pseudo_position)
            # print(range(map_size))
            # print("pseudo_ranges =")
            # print(pseudo_ranges)
            # print("observations =")
            # print(observations[t])
            # Measurement update:
            # Calculate observation probability
            observation_prob = observation_model(
                landmark_positions, observations[t],
                pseudo_ranges, observation_stdev
            )
            # print(observation_prob)
            # Calculate posterior probability
            posteriors[pseudo_position] = motion_prob * observation_prob

            # print("motion_prob =")
            # print(motion_prob)
            # print("observation_prob =")
            # print(observation_prob)
            # print("posteriors =")
            # print(posteriors[pseudo_position])
            # '''
            # print("%f\t%f\t%f" % (motion_prob,
            #                       observation_prob,
            #                       posteriors[pseudo_position])
            # )
            # '''

        # Normalize the posterior probability distribution
        posteriors = normalize_distribution(posteriors)
        # print(posteriors)
        # Update priors with posteriors
        priors = posteriors

        # Collect data to plot according to timestep.
        graph.append(posteriors)

    # Now we generate an animated plot with the saved data.
    fig, ax = plt.subplots(figsize=(10, 10), num='Markov Localization')
    bgraph = plt.bar(range(map_size), [0] * map_size)
    plt.ylim(0, 1)
    graph_animator = GraphAnimator(bgraph, graph)
    ani = animation.FuncAnimation(
        fig, graph_animator.animate, blit=True, interval=1000, repeat=False,
        frames=len(graph)
    )
    ani.save('Markov_of_yunho.gif', writer='imagemagick', fps=1, dpi=100)

    plt.show()
