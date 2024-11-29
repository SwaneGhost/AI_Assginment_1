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
def advanced_heuristic(_grid_robot_state):
    robot_location = _grid_robot_state.robot_location
    lamp_location = _grid_robot_state.lamp_location
    manhattan_distance = abs(robot_location[0] - lamp_location[0]) + abs(robot_location[1] - lamp_location[1])

    max_exploration_distance = _grid_robot_state.get_max_exploration_distance()
    carry_preference_bonus = -1 if manhattan_distance <= max_exploration_distance and _grid_robot_state.carry > 0 else 0

    # Penalty for obstacles around the robot
    obstacle_penalty = sum(1 for move, _ in _grid_robot_state.get_valid_map_movements() if _grid_robot_state.get_map_at(*move) == -1)

    # Proximity bonus for being close to the lamp
    proximity_bonus = -2 if manhattan_distance <= 2 else 0

    heuristic_value = manhattan_distance + carry_preference_bonus + obstacle_penalty + proximity_bonus

    return heuristic_value

