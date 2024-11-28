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

    # Bonus for carrying stairs, increasing as the robot gets closer to the lamp
    carry_bonus = _grid_robot_state.carry * (10 - manhattan_distance // 2)

    # Penalty for being further away from the lamp without carrying stairs
    no_carry_penalty = manhattan_distance if _grid_robot_state.carry == 0 else 0

    # Additional bonus for being at the lamp location
    at_lamp_bonus = -5 if _grid_robot_state.was_at_lamp else 0

    # Additional penalty for moving away from the lamp
    move_away_penalty = 5 if _grid_robot_state.previous_action in [PreviousAction.MOVE_LEFT, PreviousAction.MOVE_RIGHT, PreviousAction.MOVE_UP, PreviousAction.MOVE_DOWN] else 0

    return manhattan_distance - carry_bonus + no_carry_penalty + at_lamp_bonus + move_away_penalty

