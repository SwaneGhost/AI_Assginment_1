import time
from heuristics import *
from search import *


if __name__ == '__main__':

    maps = [
        [
            [0, 0, 0, 1],
            [0, 1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]

        ],
        [
            [0, 0, 0, 1],
            [0, 2, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 0]
        ]
    ]

    larger_maps = [
        [
            [0, 0, 0, 0, 0, 1, 0],
            [1, -1, 2, -1, 0, 0, 0],
            [0, -1, 0, -1, 0, 0, 0],
            [0, 0, -1, -1, -1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [0, 0, 4, 0, 0, 0, 0]
        ],
        [
            [0, 0, 0, 0, 0, 0, 0],
            [1, 4, 2, -1, 0, 0, 0],
            [0, -1, 0, -1, 0, -1, 0],
            [0, 2, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, -1, 0],
            [0, 0, 0, 0, 0, 0, 0]
        ],
        [
            [2, 0, 0, 0, 2, 0, 0],
            [1, -1, -1, 0, 1, 0, 0],
            [0, 0, 3, 0, 0, 0, -1],
            [1, 3, 1, 0, 0, 0, 0],
            [-1, -1, -1, 0, 0, 0, 0],
            [0, 0, -1, 0, 0, 0, 0]
        ]
    ]

    medium_maps = [
        [
            [0, 0, 0, 1],
            [1, -1, 2, 0],
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [0, 0, 0, 0]
        ],
        [
            [0, 0, 0, 0],
            [1, 4, 2, -1],
            [0, -1, 0, -1],
            [0, 2, 0, 0],
            [0, 0, 0, 0]
        ],
        [
            [2, 0, 0, 2],
            [1, -1, -1, 1],
            [0, 0, 3, 0],
            [1, 3, 1, 0],
            [-1, -1, -1, 0]
        ]
    ]
    robot_start_location = (0, 0)
    lamp_h = 2
    lamp_location = (5, 6)

    for map in larger_maps:
        start_state = grid_robot_state(map=map, robot_location=robot_start_location, lamp_height=lamp_h,
                                       lamp_location=lamp_location)
        start_time = time.time()
        search_result = search(start_state, base_heuristic)
        end_time = time.time() - start_time
        # runtime
        print(f"Base heuristic runtime: {end_time}")
        # solution cost
        print(f"Base heuristic solution cost: {search_result[-1].g}")

        start_time = time.time()
        search_result = search(start_state, advanced_heuristic)
        end_time = time.time() - start_time
        # runtime
        print(f"Advanced heuristic runtime: {end_time}")
        # solution cost
        print(f"Advanced heuristic solution cost: {search_result[-1].g}")
