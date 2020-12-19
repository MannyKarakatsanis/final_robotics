import numpy as np

# solves maze using backtracking
def solveMaze(maze): 
    # need 4x4 list
    path = [ [ 0 for i in range(4) ] for j in range(4) ] 
      
    if findNext(maze, 0, 0, path) == False: 
        print("No path available") 
        return False
      
    print("Path found")
    for i in path: 
        for j in i: 
            print(str(j) + " ", end="") 
        print("")
    return True
      
# recursively solve maze
def findNext(maze, x, y, path): 
    # goal has been reached
    if x == 3 and y == 3: 
        path[x][y] = 1
        return True
          
    # determine whether position is viable
    if x >= 0 and x < 4 and y >= 0 and y < 4 and maze[x][y] == 1: 
        # mark x, y as part of solution path 
        path[x][y] = 1
          
        # Move forward in x direction 
        if findNext(maze, x + 1, y, path) == True: 
            return True
              
        # Move down in y direction when moving in x direction doesn't work 
        if findNext(maze, x, y + 1, path) == True: 
            return True
          
        # if neither work, coordinate is not part of the solution
        path[x][y] = 0
        return False

if __name__ == "__main__":
    #initialize 4 x 4 maze randomly
    # maze = np.mod(np.random.permutation(4*4).reshape(4,4),2)
    maze = [[1, 0, 1, 1], 
             [1, 1, 1, 1], 
             [0, 0, 1, 0], 
             [0, 1, 1, 1]] 

    print("Input maze")
    for i in maze: 
        for j in i: 
            print(str(j) + " ", end="") 
        print("")

    solveMaze(maze)
