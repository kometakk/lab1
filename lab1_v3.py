class AstarNode:
    # Parameters:
    # coords -> tuple=>(integer, integer)
    # cost_so_far -> decimal
    # priority -> decimal
    def __init__(self, coords, cost_so_far, priority):
        self.coords = coords
        self.cost_so_far = cost_so_far
        self.priority = priority

    def getNodeNeighbors(currNode, maze):
        # maze[y][x]
        if(currNode.coords[0]>0 and currNode.coords[0])

        vert_dist = abs(self.coords[0]-otherNode.coords[0])
        horiz_dist = abs(self.coords[1]-otherNode.coords[1])
        if(vert_dist == 1 and horiz_dist == 0):
            return True
        if(vert_dist == 0 and horiz_dist == 1):
            return True
        return False

# First heuristic function: Manhattan
def hManhattan(currCoords, finishCoords):
    return ((finishCoords[0] - currCoords[0])**2 + (finishCoords[1] - currCoords[1])**2)**0.5
    """Heuristic function calculated here"""
    
def findNodeWithLowestF(frontier, finish):
    maxF = -1.0
    maxNode = frontier[i]
    for i in range(1, len(frontier)):
        currF = frontier[i].cost_so_far + hManhattan(frontier[i].coords, finish)
        if(maxF < currF):
            maxF = currF
            maxNode = frontier[i]
    return maxNode

def reconstructPath(came_from, start, finish):
    return "Not implemented yet"



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

    # frontier <- {s}; cost_so_far[s] <- 0
    frontier = [AstarNode(start, 0, 0+hManhattan(start, finish))]
    # came_from <- {}
    came_from = {}
    # while frontier is not empty do
    while(frontier != []):
        # current <- node in frontier with lowest f value
        current_node = findNodeWithLowestF(frontier, finish)
        # in current = g then
        if(current_node.coords == finish):
            # return reconstruct_path(came_from, s, g)
            return reconstructPath(came_from, start, finish)
        # remove current from frontier
        for node in frontier:
            if(node.corrds == current_node.coords):
                frontier.pop()
                break
        # foreach neighbor next of current do
        neighbors_current_node = getNodeNeighbors(current_node, maze)
        """ TODO: scan for new current_node neighbors
            and if some neighbor is not present, add
            it to the frontier. New neighbor has cost_so_far
            equal to -1"""
        for neighborNode in neighbors_current_node:
            if(current_node.isNeighbor(neighborNode) == False):
                continue
            # new_cost <- cost_so_far[current] + c(current, next)
            new_cost = current_node.cost_so_far + 1
            # if next not in cost_so_far or new_cost < cost_so_far[next] then
            if(neighborNode.cost_so_far == -1 or new_cost < neighborNode.cost_so_far):
                # cost_so_far[next] <- new_cost
                neighborNode.cost_so_far = new_cost
                # priority <- new_cost + h(next, g); add next to frontier with priority priority
                neighborNode.priority <- new_cost + hManhattan(neighborNode.coords, finish)
                # add next to frontier with priority priority
                frontier.append(neighborNode)
                # came_fron[next] <- current
                came_from.update(current_node.coords, neighborNode.coords)


        

    # Write your code here

def vizualize(viz):
    """
    Vizualization function. Shows step by step the work of the search algorithm

    Parameters:
    - viz: everything required for step-by-step vizualization
    """


# Example usage:
maze = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 0, 1, 0, 0],
    [0, 0, 0, 1, 0]
]

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