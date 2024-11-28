from grid_robot_state import grid_robot_state

def base_heuristic(_grid_robot_state):
    """
    returns the manhattan distance between the robot and the goal

    Args:
        param1 (_grid_robot_state): Current state of the search problem.

    Returns:
        int: The manhattan distance between the robot and the goal.
    """
    robot_location = _grid_robot_state.robot_location
    lamp_location = _grid_robot_state.lamp_location
    return abs(robot_location[0] - lamp_location[0]) + abs(robot_location[1] - lamp_location[1])


# TODO work on the heuristic
def advanced_heuristic(_grid_robot_state):
    """
    Returns a heuristic value that considers the Manhattan distance, the height of the stairs,
    and whether the robot is carrying stairs.

    Args:
        param1 (_grid_robot_state): Current state of the search problem.

    Returns:
        int: The heuristic value.
    """
    alpha = 1
    exploitation = exploit_heuristic(_grid_robot_state)
    exploration = explore_heuristic(_grid_robot_state)
    heuristic_value = alpha * exploitation + (1 - alpha) * exploration

    return heuristic_value

def exploit_heuristic(_grid_robot_state):
    return 0

def explore_heuristic(_grid_robot_state):
    return 0