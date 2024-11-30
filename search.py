import heapq
from search_node import search_node
from grid_robot_state import grid_robot_state

def create_open_set():
    return [], {}

def create_closed_set():
    return {}

def add_to_open(vn, open_set):
    heapq.heappush(open_set[0], vn)
    open_set[1][vn.state] = vn

def open_not_empty(open_set):
    return len(open_set[0]) > 0

def get_best(open_set):
    best = heapq.heappop(open_set[0])
    del open_set[1][best.state]
    return best

def add_to_closed(vn, closed_set):
    closed_set[vn.state] = vn

def duplicate_in_open(vn, open_set):
    if vn.state not in open_set[1]:
        return False
    existing_node = open_set[1][vn.state]
    if vn.g < existing_node.g:
        open_set[0].remove(existing_node)
        heapq.heapify(open_set[0])
        heapq.heappush(open_set[0], vn)
        open_set[1][vn.state] = vn
    return True

def duplicate_in_closed(vn, closed_set):
    return vn.state in closed_set

def search(start_state, heuristic):
    open_set = create_open_set()
    closed_set = create_closed_set()
    start_node = search_node(start_state, 0, heuristic(start_state))
    add_to_open(start_node, open_set)

    # TODO delete after testing
    states_expanded = 0

    # *****************************************************

    while open_not_empty(open_set):

        current = get_best(open_set)

        # TODO delete after testing
        states_expanded += 1
        # *****************************************************

        if grid_robot_state.is_goal_state(current.state):
            path = []
            while current:
                path.append(current)
                current = current.prev
            path.reverse()

            # TODO delete expanded states after testing
            print(f"States expanded: {states_expanded}")
            # *****************************************************

            return path

        add_to_closed(current, closed_set)

        for neighbor, edge_cost in current.get_neighbors():
            curr_neighbor = search_node(neighbor, current.g + edge_cost, heuristic(neighbor), current)
            if duplicate_in_closed(curr_neighbor, closed_set):
                continue
            if not duplicate_in_open(curr_neighbor, open_set):
                add_to_open(curr_neighbor, open_set)

    return None