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

def advanced_heuristic(_grid_robot_state):
    """
    Returns a heuristic value that considers the Manhattan distance, the height of the stairs,
    and whether the robot is carrying stairs.

    Args:
        param1 (_grid_robot_state): Current state of the search problem.

    Returns:
        int: The heuristic value.
    """
    robot_location = _grid_robot_state.robot_location
    lamp_location = _grid_robot_state.lamp_location
    lamp_height = _grid_robot_state.lamp_height
    carry = _grid_robot_state.carry

    # Manhattan distance between the robot and the lamp
    manhattan_distance = abs(robot_location[0] - lamp_location[0]) + abs(robot_location[1] - lamp_location[1])

    # Additional cost if the robot is not carrying the stairs
    if carry == 0:
        stairs_height = _grid_robot_state.get_map_at(robot_location[0], robot_location[1])
        if stairs_height > 0:
            # Cost to pick up the stairs
            pick_up_cost = 1
        else:
            # Cost to find and pick up the stairs
            pick_up_cost = manhattan_distance + 1
    else:
        pick_up_cost = 0

    # Additional cost if the robot is carrying the stairs but not at the lamp location
    if carry > 0 and robot_location != lamp_location:
        carry_cost = manhattan_distance
    else:
        carry_cost = 0

    # Total heuristic value
    heuristic_value = manhattan_distance + pick_up_cost + carry_cost

    return heuristic_value
