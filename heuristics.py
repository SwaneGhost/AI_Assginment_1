from grid_robot_state import grid_robot_state, PreviousAction


def base_heuristic(_grid_robot_state):
    """
    This heuristic is the Manhattan distance between the robot and the lamp

    :param _grid_robot_state:

    :return: The Manhattan distance between the robot and the lamp
    """
    robot_location = _grid_robot_state.robot_location
    lamp_location = _grid_robot_state.lamp_location
    return abs(robot_location[0] - lamp_location[0]) + abs(robot_location[1] - lamp_location[1])

def advanced_heuristic(_grid_robot_state):
    """
    This heuristic is magic
    When it was written only god and I knew what it was
    Now only god knows
    :param _grid_robot_state:

    :return: a magic number
    """
    # Extract key state properties
    robot_location = _grid_robot_state.robot_location
    lamp_location = _grid_robot_state.lamp_location
    carry = _grid_robot_state.carry

    # Basic Manhattan distance to prioritize proximity
    manhattan_distance = abs(robot_location[0] - lamp_location[0]) + abs(robot_location[1] - lamp_location[1])

    # Remaining height needed at the lamp's location
    current_lamp_height = _grid_robot_state.get_map_at(*lamp_location)
    height_needed = max(0, _grid_robot_state.lamp_height - (current_lamp_height + carry))

    # Penalize lack of progress (if not carrying stairs when needed)
    carry_penalty = 0 if carry > 0 else height_needed

    # Penalize obstacles in the direct path (estimated simplistically)
    obstacle_penalty = sum(
        1 for move, _ in _grid_robot_state.get_valid_map_movements()
        if _grid_robot_state.get_map_at(*move) == -1
    )

    # Combine components with weights
    heuristic_value = manhattan_distance + height_needed + carry_penalty + obstacle_penalty

    return heuristic_value