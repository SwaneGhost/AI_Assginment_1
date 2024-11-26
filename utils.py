import enum

class PreviousAction(enum.Enum):
    MOVE_RIGHT = 'move_right'
    MOVE_LEFT = 'move_left'
    MOVE_UP = 'move_up'
    MOVE_DOWN = 'move_down'
    PICK_UP = 'pick_up'
    DROP = 'drop'
    Connect = 'connect'