import queue
from search_node import search_node
from grid_robot_state import grid_robot_state

def create_open_set():
    return queue.PriorityQueue(), set()


def create_closed_set():
    return set()


def add_to_open(vn, open_set):
    open_set[0].put(vn)
    open_set[1].add(vn.state.get_state_str())


def open_not_empty(open_set):
    return not open_set[0].empty()


def get_best(open_set):
    best = open_set[0].get()
    # TODO remove from open_set[1] as well
    # something wonky is happening with them i suspect that the sets are getting very large
    return best


def add_to_closed(vn, closed_set):
    closed_set.add(vn.state)

#returns False if curr_neighbor state not in open_set or has a lower g from the node in open_set
#remove the node with the higher g from open_set (if exists)
def duplicate_in_open(vn, open_set):
    if vn.state not in open_set[1]:
        return False
    for node in list(open_set[0].queue):
        if node.state == vn.state:
            if vn.g < node.g:
                return False
            else:
                open_set[0].queue.remove(node)
                open_set[1].remove(node.state)
                return True
    return False

#returns False if curr_neighbor state not in closed_set or has a lower g from the node in closed_set
#remove the node with the higher g from closed_set (if exists)
def duplicate_in_closed(vn, closed_set):
    return vn.state in closed_set


# helps to debug sometimes..
def print_path(path):
    for i in range(len(path)-1):
        print(f"[{path[i].state.get_state_str()}]", end=", ")
    print(path[-1].state.state_str)


def search(start_state, heuristic):
    states_expanded = 0

    open_set = create_open_set()
    closed_set = create_closed_set()
    start_node = search_node(start_state, 0, heuristic(start_state))
    add_to_open(start_node, open_set)

    while open_not_empty(open_set):

        current = get_best(open_set)
        states_expanded += 1

        if grid_robot_state.is_goal_state(current.state):
            path = []
            while current:
                path.append(current)
                current = current.prev
            path.reverse()
            print(f"States expanded: {states_expanded}")
            return path

        add_to_closed(current, closed_set)

        for neighbor, edge_cost in current.get_neighbors():
            curr_neighbor = search_node(neighbor, current.g + edge_cost, heuristic(neighbor), current)
            if not duplicate_in_open(curr_neighbor, open_set) and not duplicate_in_closed(curr_neighbor, closed_set):
                add_to_open(curr_neighbor, open_set)

    return None




