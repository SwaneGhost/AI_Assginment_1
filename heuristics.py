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


    robot_location = _grid_robot_state.robot_location
    lamp_location = _grid_robot_state.lamp_location
    manhattan_distance = abs(robot_location[0] - lamp_location[0]) + abs(robot_location[1] - lamp_location[1])

    # In exploitation mode, prefer carrying larger stairs to the lamp
    if manhattan_distance <= _grid_robot_state.exploration_distance:
        if _grid_robot_state.carry != 0:
            return _grid_robot_state.lamp_height - _grid_robot_state.carry
        else:
            return manhattan_distance

    # In exploration mode, prefer getting closer to the lamp
    return manhattan_distance

