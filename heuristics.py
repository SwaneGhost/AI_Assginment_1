from grid_robot_state import grid_robot_state


def base_heuristic(_grid_robot_state):
    """
    returns the manhattan distance between the robot and the goal

    Args:
        param1 (_grid_robot_state): Current state of the search problem.

    Returns:
        int: The manhattan distance between the robot and the goal.
    """
    return abs(_grid_robot_state.robot_location[0] - _grid_robot_state.lamp_location[0]) + abs(
        _grid_robot_state.robot_location[1] - _grid_robot_state.lamp_location[1])


# thoughts:
# distance to the correct group and the carrying cost
# ???
def advanced_heuristic(_grid_robot_state):
    score = 0

    manhattan_distance = (abs(_grid_robot_state.robot_location[0] - _grid_robot_state.lamp_location[0]) +
                          abs(_grid_robot_state.robot_location[1] - _grid_robot_state.lamp_location[1]))

    # add to the score the manhattan distance between the robot and the lamp
    score += manhattan_distance

    # if the robot is carrying stairs add the height of the stairs to the score
    if _grid_robot_state.stairs_carry > 0:
        score += _grid_robot_state.stairs_carry

    # if there are stairs where the lamp is or the robot is carrying stairs where the lamp is
    # add the stairs height to the score
    if _grid_robot_state.map[_grid_robot_state.lamp_location[0]][_grid_robot_state.lamp_location[1]] > 0:
        score += _grid_robot_state.map[_grid_robot_state.lamp_location[0]][_grid_robot_state.lamp_location[1]]
    if _grid_robot_state.lamp_location == _grid_robot_state.robot_location:
        score += _grid_robot_state.stairs_carry

    return score
