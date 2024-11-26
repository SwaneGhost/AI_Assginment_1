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


# thoughts:
# distance to the correct group and the carrying cost
# ???
def advanced_heuristic(_grid_robot_state):

    robot_location = _grid_robot_state.robot_location
    lamp_location = _grid_robot_state.lamp_location
    stairs_carry = _grid_robot_state.stairs_carry

    # add to the score the manhattan distance between the robot and the lamp
    manhattan_distance = abs(robot_location[0] - lamp_location[0]) + abs(robot_location[1] - lamp_location[1])
    score = manhattan_distance

    # if the robot is carrying stairs add the height of the stairs to the score
    if stairs_carry > 0:
        score += stairs_carry

    # if there are stairs where the lamp is or the robot is carrying stairs where the lamp is
    # add the stairs height to the score
    lamp_stairs_height = _grid_robot_state.map[lamp_location[0]][lamp_location[1]]
    if lamp_stairs_height > 0:
        score += lamp_stairs_height

    if robot_location == lamp_location:
        score += stairs_carry

    return score
