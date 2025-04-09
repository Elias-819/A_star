"""
Assignment 2:Path Planning
A* Path Planning Algorithm
Auther: Dongze Li
Student ID:11530725
"""

def do_a_star(grid, start, end, display_message):
    """
    Perform A* pathfinding algorithm to find the shortest path from start to end in a grid.
    
    Args:
        grid (list of list of int): 2D grid representing the map (1 = walkable, 0 = blocked).
        start (tuple): Starting position as (row, column).
        end (tuple): Target position as (row, column).
        display_message (function): Function to display messages (used for debugging or logging).

    Returns:
        list of tuple: Shortest path from start to end as a list of (row, column) coordinates.
                       Returns an empty list if no path is found.
    """
    
    # Get the number of rows and columns in the grid
    COL = len(grid)     # Number of rows in the grid
    ROW = len(grid[0])  # Number of columns in the grid
    
    # Define the possible movement directions
    # The motion model only allow four directions of movement.
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Define the heuristic function (Euclidean distance)
    def heuristic(a, b):
        """
        Calculate the heuristic value between two points using the Euclidean distance formula.

        Args:
            a (tuple): First point as (row, column).
            b (tuple): Second point as (row, column).

        Returns:
            float: Euclidean distance between point a and point b.
        """
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2) ** 0.5

    # Define a class to represent a node in the search grid
    class Node:
        def __init__(self, position, g, h, parent=None):
            """
            Initialize a node for use in the A* algorithm.

            Args:
                position (tuple): Node position in the grid as (row, column).
                g (float): Cost from the start node to this node.
                h (float): Estimated cost from this node to the end node (using heuristic).
                parent (Node): Parent node (used for reconstructing the path).
            """
            self.position = position
            self.g = g                  # Cost from start node to current node
            self.h = h                  # Estimated cost from current node to end node
            self.f = g + h              # Total estimated cost (f = g + h)
            self.parent = parent        # Reference to the parent node

        # Define comparison for priority queue (min-heap behavior)
        def __lt__(self, other):
            """
            Define how nodes are compared based on the f value.
            This allows the priority queue to sort nodes based on the lowest f value.
            """
            return self.f < other.f

    # Initialize the open and closed lists:
    open_list = []   # Open list: nodes to be evaluated
    closed_list = set()   # Closed list: nodes already evaluated

    # Create the starting node and add it to the open list
    start_node = Node(start, 0, heuristic(start, end))
    open_list.append(start_node)

    # Main loop — keeps running until there are no more nodes to evaluate
    while open_list:
        # Find the node with the lowest f value (best candidate) in the open list
        current_node = min(open_list, key=lambda node: node.f)
        open_list.remove(current_node)
        current_position = current_node.position

        # Check if the goal has been reached
        if current_position == end:
            # Goal reached — backtrack to reconstruct the path
            path = []
            while current_node:
                path.append(current_node.position)  # Add node to the path
                current_node = current_node.parent  # Move to parent node
            path.reverse()  # Reverse to get path from start to end
            display_message("Path found!")
            return path

        # Add the current node to the closed list (mark as processed)
        closed_list.add(current_position)

        # Explore neighboring nodes (up, down, left, right)
        for direction in directions:
            # Compute the position of the neighboring node
            neighbor_position = (current_position[0] + direction[0], current_position[1] + direction[1])

            # Ensure the neighboring node is within grid bounds
            if 0 <= neighbor_position[0] < COL and 0 <= neighbor_position[1] < ROW:
                # Skip if the neighboring node is a wall (value = 0) or already processed
                if grid[neighbor_position[0]][neighbor_position[1]] == 0 or neighbor_position in closed_list:
                    continue

                # Calculate new g (cost from start to neighbor)
                # Each horizontal or vertical cell movement contributes a cost of 1 to the path length
                g_cost = current_node.g + 1
                # Calculate heuristic (estimated cost from neighbor to goal)
                h_cost = heuristic(neighbor_position, end)
                # Create a new node for the neighbor
                neighbor_node = Node(neighbor_position, g_cost, h_cost, current_node)

                # Check if this neighbor node is already in the open list with a lower f value
                if not any(neighbor.position == neighbor_position and neighbor.f <= neighbor_node.f for neighbor in open_list):
                    # If not, add it to the open list for future evaluation
                    open_list.append(neighbor_node)

        # Display the current node being evaluated (for debugging)
        display_message(f"Evaluating node: {current_position}")

    # If we exhaust the open list without finding the goal, there is no path
    display_message("No path found!")
    return []
