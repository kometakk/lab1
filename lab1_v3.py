class AstarNode:
    # Parameters:
    # coords -> tuple=>(integer, integer)
    # cost_so_far -> decimal
    # priority -> decimall
    def __init__(self, coords, cost_so_far, priority):
        self.coords = coords
        self.cost_so_far = cost_so_far
        self.priority = priority

# First heuristic function: Manhattan
def hEuclidean(currCoords, finishCoords):
    return ((finishCoords[0] - currCoords[0])**2 + (finishCoords[1] - currCoords[1])**2)**0.5
    """Heuristic function calculated here"""

def hManhattan(currCoords, finishCoords):
    return abs(finishCoords[0] - currCoords[0]) + abs(finishCoords[1] - currCoords[1])
    

def reconstructPath(came_from, start, finish):
    path = []
    current = finish
    num_steps = 0
    while current != start:
        path.append(current)
        current = came_from.get(current)
        num_steps += 1
        if current is None:
            # No path found
            return [], -1
    path.append(start)
    path.reverse()
    return path, num_steps


def vizCoordsToString(coords):
    return "(" + str(coords[0]) + ", " + str(coords[1]) + ") "


def astar(maze, start, finish):
    
    """
    A* search

    Parameters:
    - maze: The 2D matrix that represents the maze with 0 represents emptry space and 1 represents a wall
    - start: A tuple with the coordinates of starting position
    - finish: A tuple with the coordinates of finishing position

    Returns:
    - Number of steps from start to finish, equals -1 if the path is not found
    - Viz - everything required for step-by-step vizualization
    
    """
    # Write your code here

    viz = ""
    # Variables:
    # all_nodes: Dictionary<coords, AStarNode>
    # frontier: Array<coords>
    # came_from: Dictionary<coords, coords>

    # It will be used to store every node info
    # all_nodes[(int, int)] -> AstarNode
    all_nodes = {}

    # came_from <- {}
    # came_from[some_node_coords] == coords_of_node_from_which_some_node_came_from
    came_from = {}


    # frontier <- {s}; cost_so_far[s] <- 0
    all_nodes[start] = AstarNode(start, 0, 0+hEuclidean(start, finish))
    frontier = [start]
    

    # while frontier is not empty do
    while(frontier != []):
        # current <- node in frontier with lowest f value
        temp_min_node = all_nodes[frontier[0]]
        minF = temp_min_node.cost_so_far + hEuclidean(temp_min_node.coords, finish)
        for i in range(1, len(frontier)):
            temp_loop_node = all_nodes[frontier[i]]
            currF = temp_loop_node.cost_so_far + hEuclidean(temp_loop_node.coords, finish)
            if(minF > currF):
                minF = currF
                temp_min_node = temp_loop_node

        current_node = temp_min_node

        # in current = g then
        if(current_node.coords == finish):
            # return reconstruct_path(came_from, s, g)
            path, num_steps = reconstructPath(came_from, start, finish)
            ####
            viz += "\nFinal path:\n"
            viz += str(path) + "\n"
            viz += "Number of steps = " + str(num_steps)
            ####
            return num_steps, viz
        # remove current from frontier
        for i in range(0, len(frontier)):
            if(frontier[i] == current_node.coords):
                frontier.pop(i)
                break
        # foreach neighbor next of current do

        ####
        viz += "Current node: "
        viz += "coords = " + vizCoordsToString(current_node.coords) + ", "
        viz += "cost_so_far = " + str(current_node.cost_so_far) + "\n"
        ####

        neighbor_nodes_coords = []
        #self.coords[0] - x == column   |== maze[][x]
        #self.coords[1] - y == row      |== maze[y][]
        # North
        # y > 0 and maze[y-1][] == 0

        for neighbor_vector in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neigh_coords = (current_node.coords[0] + neighbor_vector[0], current_node.coords[1] + neighbor_vector[1])
            is_neigh_in_maze_verti = neigh_coords[1] >= 0 and neigh_coords[1] < len(maze)
            is_neigh_in_maze_horiz = neigh_coords[0] >= 0 and neigh_coords[0] < len(maze[0])
            if(is_neigh_in_maze_verti and is_neigh_in_maze_horiz and maze[neigh_coords[1]][neigh_coords[0]] == 0):
                neighbor_nodes_coords.append(neigh_coords)

        ####
        viz += "Neighbors: "
        for neighbor_coords in neighbor_nodes_coords:
            viz += vizCoordsToString(neighbor_coords) + " "
        viz += "\n"
        ####

        for neighbor_coords in neighbor_nodes_coords:
            # new_cost <- cost_so_far[current] + c(current, next)
            new_cost = current_node.cost_so_far + 1
            # if next not in cost_so_far or new_cost < cost_so_far[next] then
            if(neighbor_coords not in all_nodes or new_cost < all_nodes[neighbor_coords].cost_so_far):
                

                # cost_so_far[next] <- new_cost
                # priority <- new_cost + h(next, g); add next to frontier with priority priority
                newNode = AstarNode(neighbor_coords, new_cost, new_cost + hEuclidean(neighbor_coords, finish))
                
                ####
                viz += "New node: " + vizCoordsToString(newNode.coords) + " " + str(newNode.cost_so_far) + " " + str(newNode.priority) + "\n"
                ####

                all_nodes[neighbor_coords] = newNode
                
                # add next to frontier with priority priority
                frontier.append(newNode.coords)
                # came_fron[next] <- current
                came_from[neighbor_coords] = current_node.coords
        
        viz += "\n\n"

    # If no path found:
    return -1, viz


def vizualize(viz):
    print("\n\n[[[Vizualisation]]]")
    print(viz)


# Example usage:
maze = [
# x| 0  1  2  3  4
    [0, 1, 0, 0, 0], # 0 \y
    [0, 1, 0, 1, 0], # 1
    [0, 0, 0, 1, 0], # 2
    [1, 0, 1, 0, 0], # 3
    [0, 0, 0, 1, 0]  # 4
]
# (x, y) == (column, row)
# maze[y][x]
start_position = (0, 0)
finish_position = (4, 4)

num_steps, viz = astar(maze, start_position, finish_position)

# Print number of steps in path
if num_steps != -1:
    print(f"Path from {start_position} to {finish_position} using A* is {num_steps} steps.")

else:
    print(f"No path from {start_position} to {finish_position} exists.")

# Vizualize algorithm step-by-step even if the path was not found
vizualize(viz)