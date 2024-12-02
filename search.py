import heapq
from search_node import search_node
from grid_robot_state import grid_robot_state

def create_open_set():
    """
    Create an open set for the A* search algorithm

    :return: an empty list and an empty dictionary
    """
    return [], {}

def create_closed_set():
    """
    Create a closed set for the A* search algorithm

    :return: an empty dictionary
    """
    return {}

def add_to_open(vn, open_set):
    """
    Add a search node to the open set

    :param vn: the search node to add
    :param open_set: The open set (min_heap, dictionary) to add the search node to
    """
    heapq.heappush(open_set[0], vn)
    open_set[1][vn.state] = vn

def open_not_empty(open_set):
    """
    Check if the open set is not empty

    :param open_set: The open set (min_heap, dictionary) to add the search node to
    :return: If the open set is not empty
    """
    return len(open_set[0]) > 0

def get_best(open_set):
    """
    Get the best search node from the open set by what is defined in the search node class

    :param open_set: The open set (min_heap, dictionary) to add the search node to
    :return: The best search node
    """
    best = heapq.heappop(open_set[0])
    del open_set[1][best.state]
    return best

def add_to_closed(vn, closed_set):
    """
    Add a search node to the closed set
    :param vn: The search node to add
    :param closed_set: The closed set to add the search node to
    """
    closed_set[vn.state] = vn

def duplicate_in_open(vn, open_set):
    """
    Check if a search node is in the open set and if it is, update it if the new search node is better

    :param vn: The search node to check
    :param open_set: The open set (min_heap, dictionary) to add the search node to
    :return: If there is a better node in the open set
    """
    # The node was never added to the open set
    if vn.state not in open_set[1]:
        return False
    # The node is already in the open set
    # Get the existing node from the open set
    existing_node = open_set[1][vn.state]
    # If the new node is better, update the existing node
    if vn.g < existing_node.g:
        open_set[0].remove(existing_node)   # remove the existing node from the heap
        heapq.heapify(open_set[0])          # re-heapify the heap
        heapq.heappush(open_set[0], vn)     # add the new node to the heap
        open_set[1][vn.state] = vn          # update the dictionary
    # Return that there is a node and no need to add it
    return True

def duplicate_in_closed(vn, closed_set):
    """
    Check if a search node is in the closed set
    :param vn: The search node to check
    :param closed_set: The closed set to check
    :return: True if the search node is in the closed set
    """
    return vn.state in closed_set

def search(start_state, heuristic):
    """
    Perform an A* search on the given start state with the given heuristic

    :param start_state: The start state of the search
    :param heuristic: The heuristic to use in the search
    :return: The path from the start state to the goal state
    """
    # Initialize the open and closed sets
    open_set = create_open_set()        # (min_heap, dictionary)
    closed_set = create_closed_set()    # dictionary
    # Create the start node
    start_node = search_node(start_state, 0, heuristic(start_state))
    # Add the start node to the open set
    add_to_open(start_node, open_set)

    while open_not_empty(open_set):
        # Get the best node from the open set
        current = get_best(open_set)
        # Check if the current node is the goal state
        if grid_robot_state.is_goal_state(current.state):
            path = []
            while current:
                path.append(current)
                current = current.prev
            # Reverse the path to get the correct order
            path.reverse()
            return path
        # Add the current node to the closed set
        add_to_closed(current, closed_set)
        # Get the neighbors of the current node
        for neighbor, edge_cost in current.get_neighbors():
            curr_neighbor = search_node(neighbor, current.g + edge_cost, heuristic(neighbor), current)
            # Check if the neighbor is in the closed set, if so skip it
            if duplicate_in_closed(curr_neighbor, closed_set):
                continue
            # Check if the neighbor is in the open set or better, if not add it
            if not duplicate_in_open(curr_neighbor, open_set):
                add_to_open(curr_neighbor, open_set)

    return None