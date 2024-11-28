import enum
from collections import defaultdict

class PreviousAction(enum.Enum):
    MOVE_RIGHT = 1
    MOVE_LEFT = 2
    MOVE_UP = 3
    MOVE_DOWN = 4
    PICK_UP = 5
    DROP = 6
    CONNECT = 7
class grid_robot_state:
    __slots__ = ['robot_location', 'map', 'lamp_height', 'lamp_location', 'carry', 'previous_action', 'map_changes','exploration_distance']

    def __init__(self, robot_location, map=[], lamp_height=-1, lamp_location=(-1, -1), exploration_distance= -1):
        self.robot_location = tuple(robot_location)
        self.map = map
        self.lamp_height = int(lamp_height)
        self.lamp_location = tuple(lamp_location)
        self.carry = 0
        self.previous_action = None
        self.map_changes = {}
        self.exploration_distance = exploration_distance if exploration_distance != -1 else grid_robot_state.find_exploration_distance(map, lamp_location, lamp_height)


    @staticmethod
    def is_goal_state(_grid_robot_state):
        return (_grid_robot_state.robot_location == _grid_robot_state.lamp_location and
                _grid_robot_state.lamp_height == _grid_robot_state.get_map_at(_grid_robot_state.robot_location[0], _grid_robot_state.robot_location[1]))

    def get_neighbors(self):
        for neighbor, direction in self.get_valid_map_movements():
            if (direction == PreviousAction.MOVE_UP and self.previous_action == PreviousAction.MOVE_DOWN) or \
                    (direction == PreviousAction.MOVE_DOWN and self.previous_action == PreviousAction.MOVE_UP) or \
                    (direction == PreviousAction.MOVE_LEFT and self.previous_action == PreviousAction.MOVE_RIGHT) or \
                    (direction == PreviousAction.MOVE_RIGHT and self.previous_action == PreviousAction.MOVE_LEFT):
                continue

            new_state = grid_robot_state(neighbor, self.map, self.lamp_height, self.lamp_location, self.exploration_distance)
            new_state.set_carry(self.carry)
            new_state.set_previous_action(direction)
            new_state.set_map_changes(self.map_changes)
            yield new_state, 1 + self.carry

        # Pick up stairs
        if (self.carry == 0 and
                self.get_map_at(self.robot_location[0], self.robot_location[1]) > 0 and
                    self.previous_action != PreviousAction.DROP):
            new_state = grid_robot_state(self.robot_location, self.map, self.lamp_height, self.lamp_location, self.exploration_distance)
            new_state.set_carry(self.get_map_at(self.robot_location[0], self.robot_location[1]))
            new_state.set_previous_action(PreviousAction.PICK_UP)
            new_state.set_map_changes(self.map_changes.copy())
            new_state.add_map_change(self.robot_location[0], self.robot_location[1], 0)
            yield new_state, 1

        # Carry stairs
        if self.carry != 0:
            # Drop the stairs
            if (self.get_map_at(self.robot_location[0], self.robot_location[1]) == 0 and
                    self.previous_action != PreviousAction.PICK_UP):
                new_state = grid_robot_state(self.robot_location, self.map, self.lamp_height, self.lamp_location, self.exploration_distance)
                new_state.set_carry(0)
                new_state.set_previous_action(PreviousAction.DROP)
                new_state.set_map_changes(self.map_changes.copy())
                new_state.add_map_change(self.robot_location[0], self.robot_location[1], self.carry)
                yield new_state, 1
            elif self.get_map_at(self.robot_location[0], self.robot_location[1]) != 0:   # Connect the stairs
                # Don't check if the combination is over the limit
                if not self.get_map_at(self.robot_location[0], self.robot_location[1]) + self.carry > self.lamp_height:
                    new_state = grid_robot_state(self.robot_location, self.map, self.lamp_height, self.lamp_location, self.exploration_distance)
                    new_state.set_carry(self.carry + self.get_map_at(self.robot_location[0], self.robot_location[1]))
                    new_state.set_previous_action(PreviousAction.CONNECT)
                    new_state.set_map_changes(self.map_changes.copy())
                    new_state.add_map_change(self.robot_location[0], self.robot_location[1], 0)
                    yield new_state, 1


    def __hash__(self):
        return hash((self.robot_location, self.carry,
                     frozenset(self.map_changes.items()),
                     self.previous_action))

    def __eq__(self, other):
        return (self.robot_location == other.robot_location and
                self.carry == other.carry and
                self.map_changes == other.map_changes and
                self.previous_action == other.previous_action)

    def get_state_str(self):
        state_str = ""
        for i in range(len(self.map)):
            for j in range(len(self.map[0])):
                if (i, j) == self.robot_location:
                    state_str += "R "  # Robot
                elif (i, j) == self.lamp_location:
                    state_str += "L "  # Lamp
                elif self.map[i][j] == -1:
                    state_str += "X "  # Obstacle
                elif self.get_map_at(i,j) > 0:
                    state_str += f"{self.get_map_at(i,j)} "  # Stairs height
                else:
                    state_str += ". "  # Empty space
            state_str += '\n'
        return state_str

    def set_carry(self, carry):
        self.carry = carry

    def set_previous_action(self, action):
        self.previous_action = action

    def set_map_changes(self, changes):
        self.map_changes = changes

    def get_map_at(self, row, col):
        change = self.map_changes.get((row, col), None)
        if change is not None:
            return change
        return self.map[row][col]

    def add_map_change(self, row, col, value):
        self.map_changes[(row, col)] = value
        if self.map[row][col] == value:
            self.map_changes.pop((row, col))

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
            yield (rows - 1, columns), PreviousAction.MOVE_UP

        # Check down
        if rows < len(self.map) - 1 and self.map[rows + 1][columns] != -1:
            yield (rows + 1, columns), PreviousAction.MOVE_DOWN

        # Check left
        if columns > 0 and self.map[rows][columns - 1] != -1:
            yield (rows, columns - 1), PreviousAction.MOVE_LEFT

        # Check right
        if columns < len(self.map[0]) - 1 and self.map[rows][columns + 1] != -1:
            yield (rows, columns + 1), PreviousAction.MOVE_RIGHT

# TODO Delete if not needed for heuristics


    @staticmethod
    def find_exploration_distance(map, lamp_location, lamp_height):
        """
        Finds the Manhattan distance from the lamp within which there is a combination of stairs
        that can be combined to reach the lamp height.

        Returns:
            int: The exploration distance.
        """

        stairs_found = {}
        found_set = False
        distance = 0

        while found_set is False:
            distance += 1
            for i in range(-distance, distance + 1):
                for j in range(-distance, distance + 1):
                    if abs(i) + abs(j) == distance:
                        row, col = lamp_location[0] + i, lamp_location[1] + j
                        if 0 <= row < len(map) and 0 <= col < len(map[0]):
                            if map[row][col] > 0:
                                stairs_found[(row, col)] = map[row][col]
                                lamp_height -= map[row][col]
                                if grid_robot_state.subset_sum_exists(list(stairs_found.values()), lamp_height):
                                    found_set = True
                                    break

        return distance

    @staticmethod
    def subset_sum_exists(nums, target):
        """
        Checks if there exists a subset in nums that sums up to target.

        Args:
            nums (list): List of integers.
            target (int): Target sum.

        Returns:
            bool: True if such a subset exists, False otherwise.
        """
        n = len(nums)
        dp = [False] * (target + 1)
        dp[0] = True

        for num in nums:
            for j in range(target, num - 1, -1):
                dp[j] = dp[j] or dp[j - num]

        return dp[target]