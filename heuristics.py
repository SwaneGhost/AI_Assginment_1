from grid_robot_state import grid_robot_state, PreviousAction


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
'''def advanced_heuristic(_grid_robot_state):
    robot_location = _grid_robot_state.robot_location
    lamp_location = _grid_robot_state.lamp_location
    manhattan_distance = abs(robot_location[0] - lamp_location[0]) + abs(robot_location[1] - lamp_location[1])
    lamp_height_left = _grid_robot_state.lamp_height - _grid_robot_state.get_map_at(*lamp_location)
    max_exploration_distance = _grid_robot_state.get_max_exploration_distance()

    # Adjust carry preference bonus
    carry_preference_bonus = 0 * lamp_height_left if manhattan_distance <= max_exploration_distance + 5 and _grid_robot_state.carry > 0 else 0

    # Adjust obstacle penalty
    obstacle_penalty = sum(0 for move, _ in _grid_robot_state.get_valid_map_movements() if _grid_robot_state.get_map_at(*move) == -1)

    # Adjust proximity bonus
    proximity_bonus = 0 if manhattan_distance <= 2 else 0

    heuristic_value = manhattan_distance + carry_preference_bonus + obstacle_penalty + proximity_bonus

    return min(max(heuristic_value, 0), _grid_robot_state.lamp_height - _grid_robot_state.get_map_at(*lamp_location) + 1 + manhattan_distance)
'''

def advanced_heuristic(_grid_robot_state):
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