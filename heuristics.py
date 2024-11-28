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

    # Calculate the maximum exploration distance
    max_exploration_distance = _grid_robot_state.get_max_exploration_distance()

    # Penalty for being further away from the lamp without carrying stairs
    no_carry_penalty = manhattan_distance if _grid_robot_state.carry == 0 else 0

    # Bonus for carrying if closer to the lamp than the maximum exploration distance
    carry_preference_bonus = -2 * manhattan_distance if manhattan_distance <= max_exploration_distance and _grid_robot_state.carry > 0 else 0

    # Calculate the heuristic value
    heuristic_value = manhattan_distance + no_carry_penalty + carry_preference_bonus

    return heuristic_value

