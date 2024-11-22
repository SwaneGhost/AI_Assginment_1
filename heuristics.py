from grid_robot_state import grid_robot_state


def base_heuristic(_grid_robot_state):
    """
    returns the manhattan distance between the robot and the goal

    Args:
        param1 (_grid_robot_state): Current state of the search problem.

    Returns:
        int: The manhattan distance between the robot and the goal.
    """
    return abs(_grid_robot_state.robot_location[0] - _grid_robot_state.lamp_location[0]) + abs(_grid_robot_state.robot_location[1] - _grid_robot_state.lamp_location[1])


# thoughts:
# distance to the correct group and the carrying cost
# ???
def advanced_heuristic(_grid_robot_state):
    return 0
