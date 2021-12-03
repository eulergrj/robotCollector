from warnings import warn

class Node:

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end):

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)
    
    # Adding a stop condition
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 2

    # what squares do we search
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0))

    # Loop until you find the end
    while len(open_list) > 0:
        outer_iterations += 1
        
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        if outer_iterations > max_iterations:
            # if we hit this point return the path such as it is
            # it will not contain the destination
            warn("giving up on pathfinding too many iterations")
            return return_path(current_node)

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            return return_path(current_node)

        # Generate children
        children = []
        
        for new_position in adjacent_squares:  # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            within_range_criteria = [
                node_position[0] > (len(maze) - 1),
                node_position[0] < 0,
                node_position[1] > (len(maze[len(maze) - 1]) - 1),
                node_position[1] < 0,
            ]
            
            if any(within_range_criteria):
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            
            # Child is on the closed list
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + \
                      ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            if len([open_node for open_node in open_list if child == open_node and child.g > open_node.g]) > 0:
                continue

            # Add the child to the open list
            open_list.append(child)

# a* algorithm end


# Robot box collecting function

# initialize variables
obstacles = []
boxes = []
robot = []
storage = []
capacity = 0
currentLoad = 0
finalPath = []

def run(data):
    global obstacles, boxes, robot, storage, capacity, currentLoad, finalPath

    boxes = data['boxes']
    obstacles = data['obstacles']
    robot = data['robot']
    storage = data['storage']
    capacity = data['capacity']

    boxCount = len(boxes)
    currentLoad = 0
    finalPath = []
    end = False
    
    # initialize 15x15 maze
    maze = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]

    # Load obstacles from frontend into maze
    for o in data['obstacles']:
        x = o[0]
        y = o[1]
        maze[x][y] = 1

    # func. to find shortest path from list of paths
    def findShortest(arr):
        shortest = []
        for ar in arr:
            
            size = len( ar['path'] )

            if(len(shortest) == 0 ):
                shortest = ar
            
            current = len( shortest['path'] )

            if(size <= current): 
                shortest = ar

        return shortest
        

    # Get path for next Box
    def getBox():

        global robot, boxes, currentLoad, finalPath

        boxPaths = []
        # iterate between existing boxes
        for k, v in enumerate(boxes):
            x = v[0]
            y = v[1]

            
            start = (robot[0], robot[1])
            end = (x,y)
            path = astar(maze, start, end)

            box = {
                'index': k,
                'pos': [x,y],
                'path': path
            }

            boxPaths.append(box)

        # from paths found, select the shortest
        find = findShortest(boxPaths)
        finalPath.append(find['path'])

        # Update robot position to position of the selected box
        robot = [find['pos'][0], find['pos'][1]]

        # Remove selected box from list of boxes
        boxes.pop(find['index'])    

        # Update current robot load
        currentLoad = currentLoad + 1


    # Get path to storage
    def goStorage():
        global robot, storage, finalPath, currentLoad

        start = (robot[0], robot[1])
        end = (storage[0],storage[1])
        spath = astar(maze, start, end)

        # Update robot position with storage position
        robot = [storage[0], storage[1]]

        # Append path to finalpath
        finalPath.append(spath)

        # Reset robot load
        currentLoad = 0
            

    # Untill all boxes have been picked up, continue searching for new ones
    while len(boxes) > 0:
        # If robot's current load is at max capacity, go to storage to unload
        if(currentLoad == capacity):
            goStorage()
        else:
            # if not on full capacity, go fetch next box
            getBox()

    # after getting all boxes, go to storage one last time
    goStorage()

    res = {
        'path': finalPath
    }

    # Sends final path back to frontend
    return res