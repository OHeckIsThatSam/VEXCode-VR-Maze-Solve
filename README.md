# VEXCode-VR-Maze-Solve
A maze solving python script for VEXcode VR.

## Execution order
1. Recursively maps maze until goal is found. Generates a graph data structure as a map of connected tiles.
2. Runs A* algorithm on the graph to find shortest path. Generates a list of Nodes with the shortest path being the connections between them.
3. Parses the list of Nodes from the destination to the start as a list of tuples (coordinate changes) for the robot to follow.
4. Moves the robot from the start to the destination based on the coordinate changes.

## Sources

#### A* Search
https://www.geeksforgeeks.org/a-search-algorithm/
https://en.wikipedia.org/wiki/A*_search_algorithm

#### Maze Stuff
https://en.wikipedia.org/wiki/Maze-solving_algorithm
https://medium.com/nerd-for-tech/graph-traversal-in-python-a-algorithm-27c30d67e0d0
https://stackoverflow.com/questions/3097556/programming-theory-solve-a-maze
