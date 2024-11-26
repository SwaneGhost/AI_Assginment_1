

class grid_robot_state:
    # you can add global params
    stairs_carry = 0
    picked_up = False
    put_down = False

    def __init__(self, robot_location, map=None, lamp_height=-1, lamp_location=(-1, -1)):
        """
        Initialize the state of the robot.

        Args:
            robot_location: The current location of the robot.
            map: The map of the environment.
            lamp_height: The height of the lamp.
            lamp_location: The location of the lamp.
        """
        self.robot_location = robot_location
        self.map = map
        self.lamp_height = lamp_height
        self.lamp_location = lamp_location

    @staticmethod
    def is_goal_state(_grid_robot_state):
        """
        Check if the given state is the goal state.

        Args:
             _grid_robot_state: Current state of the search problem.

        Returns:
            bool: True if the robot is at the lamp location with necessary stairs height, False otherwise.

            (_grid_robot_state.robot_location == _grid_robot_state.lamp_location and
                (_grid_robot_state.stairs_carry == _grid_robot_state.lamp_height or
                 _grid_robot_state.map[_grid_robot_state.robot_location[0]][_grid_robot_state.robot_location[1]] ==
                 _grid_robot_state.lamp_height))
        """

        # we suspect the stairs must be let down on the goal state

        return (_grid_robot_state.robot_location == _grid_robot_state.lamp_location and _grid_robot_state.lamp_height ==
                _grid_robot_state.map[_grid_robot_state.robot_location[0]][_grid_robot_state.robot_location[1]])

    def get_neighbors(self):
        """
        Generate all possible neighboring states from the current state.

        This method calculates all valid movements from the current robot location,
        and generates new states based on whether the robot is carrying stairs or not.

        Yields:
            tuple: A tuple containing a new `grid_robot_state` object and the cost to reach that state.
        """
        # calculate moving to all valid directions
        valid_movements = self.get_valid_map_movements()
        # yield the new states
        for movement in valid_movements:
            new_state = grid_robot_state(movement, self.map, self.lamp_height, self.lamp_location)
            new_state.set_stairs_carry(self.stairs_carry)
            yield (new_state, 1 + self.stairs_carry)

        # if not carrying stairs
        if self.stairs_carry == 0:
            # check if there are stairs under me
            if self.map[self.robot_location[0]][self.robot_location[1]] != 0:
                # pick up the stairs
                new_map = [row[:] for row in self.map]
                new_map[self.robot_location[0]][self.robot_location[1]] = 0
                new_state = grid_robot_state(self.robot_location, new_map, self.lamp_height, self.lamp_location)
                new_state.set_stairs_carry(self.map[self.robot_location[0]][self.robot_location[1]])
                yield (new_state, 1)

        # if carrying stairs
        if self.stairs_carry != 0:
            if self.map[self.robot_location[0]][self.robot_location[1]] == 0:  # no stairs under me
                # put the stairs down
                new_map = [row[:] for row in self.map]
                new_map[self.robot_location[0]][self.robot_location[1]] = self.stairs_carry
                new_state = grid_robot_state(self.robot_location, new_map, self.lamp_height, self.lamp_location)
                new_state.set_stairs_carry(0)
                yield (new_state, 1)
            else:
                # check if the stairs under me + the stairs I carry is less than or equal to the lamp height
                if self.map[self.robot_location[0]][self.robot_location[1]] + self.stairs_carry <= self.lamp_height:
                    # add the stairs to the stairs I am carrying and remove the stairs from the map
                    new_map = [row[:] for row in self.map]
                    new_map[self.robot_location[0]][self.robot_location[1]] = 0
                    new_state = grid_robot_state(self.robot_location, new_map, self.lamp_height, self.lamp_location)
                    new_state.set_stairs_carry(
                        self.stairs_carry + self.map[self.robot_location[0]][self.robot_location[1]])
                    yield (new_state, 1)

    # helper functions

    def get_state_str(self):
        """
        Print the state of the map, including the lamp, obstacles, and the robot.

        Returns:
            str: A string representation of the current state of the map.
        """
        state_str = ""
        for i in range(len(self.map)):
            for j in range(len(self.map[0])):
                if (i, j) == self.robot_location:
                    state_str += "R "  # Robot
                elif (i, j) == self.lamp_location:
                    state_str += "L "  # Lamp
                elif self.map[i][j] == -1:
                    state_str += "X "  # Obstacle
                elif self.map[i][j] > 0:
                    state_str += f"{self.map[i][j]} "  # Stairs height
                else:
                    state_str += ". "  # Empty space
            state_str += '\n'
        return state_str

    def set_stairs_carry(self, stairs_carry):
        if isinstance(stairs_carry, list):
            print("Error: stairs_carry is a list")
        self.stairs_carry = stairs_carry

    def get_valid_map_movements(self):
        """
        Generate valid movements as (row, column) tuples.

        This method checks if the robot is within the bounds of the map and if there are no obstacles
        to the right, left, up, and down.

        Yields:
            tuple: A tuple representing a valid movement as (row, column).
        """
        rows, columns = self.robot_location

        # Check up
        if rows > 0 and self.map[rows - 1][columns] != -1:
            yield (rows - 1, columns)

        # Check down
        if rows < len(self.map) - 1 and self.map[rows + 1][columns] != -1:
            yield (rows + 1, columns)

        # Check left
        if columns > 0 and self.map[rows][columns - 1] != -1:
            yield (rows, columns - 1)

        # Check right
        if columns < len(self.map[0]) - 1 and self.map[rows][columns + 1] != -1:
            yield (rows, columns + 1)

    def __eq__(self, other):
        return (self.robot_location == other.robot_location and self.map == other.map and
                self.lamp_height == other.lamp_height and self.lamp_location == other.lamp_location and
                self.stairs_carry == other.stairs_carry)

    def __hash__(self):
        return hash((self.robot_location, tuple(map(tuple, self.map)),
                     self.lamp_height, self.lamp_location, self.stairs_carry))
