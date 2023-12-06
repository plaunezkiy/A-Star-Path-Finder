import heapq
from typing import List, Dict, Tuple, T, Set

class PriorityQueue:
    def __init__(self):
        self.elements: list[tuple[float, T]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]


maze = [
    list("############################################################"),
    list("#..........................................................#"),
    list("#.............................#............................#"),
    list("#.............................#............................#"),
    list("#.............................#............................#"),
    list("#.......S.....................#............................#"),
    list("#.............................#............................#"),
    list("#.............................#............................#"),
    list("#.............................#............................#"),
    list("#.............................#............................#"),
    list("#.............................#............................#"),
    list("#.............................#............................#"),
    list("#.............................#............................#"),
    list("#######.#######################################............#"),
    list("#....#........#............................................#"),
    list("#....#........#............................................#"),
    list("#....##########............................................#"),
    list("#..........................................................#"),
    list("#..........................................................#"),
    list("#..........................................................#"),
    list("#..........................................................#"),
    list("#..........................................................#"),
    list("#...............................##############.............#"),
    list("#...............................#........G...#.............#"),
    list("#...............................#............#.............#"),
    list("#...............................#............#.............#"),
    list("#...............................#............#.............#"),
    list("#...............................###########..#.............#"),
    list("#..........................................................#"),
    list("#..........................................................#"),
    list("############################################################"),
]

# maze = [
#     list("#####"),
#     list("#S..#"),
#     list("#..G#"),
#     list("#####"),
# ]


class Cell:
    x = None
    y = None
    f = None

    def __init__(self, y, x):
        self.row = y
        self.col = x
        self.f = float("inf")
    
    def __gt__(self, other):
        return self.f > other.f
    
    def __str__(self):
        return f'{self.row}, {self.col}, {self.f}'
    
    def __repr__(self):
        return f'{self.row}-{self.col}'
    
    def __eq__(self, other):
        return self.row == other.row and self.col == other.col

    def __hash__(self):
        return hash(f'{self.row}-{self.col}')


class AStar:
    dirs = ((1, 0), (0, 1), (-1, 0), (0, -1))
    # discovered nodes
    openSet: PriorityQueue
    closedSet: Set
    # chapest preceeding node
    cameFrom: Dict[Cell, Cell]
    path: List[Cell]
    start: Cell
    goal: Cell

    def __init__(self, maze):
        self.maze = maze
        for i in range(len(maze)):
            print(maze[i])
            for j in range(len(maze[0])):
                if maze[i][j] == "S":
                    self.start = Cell(i, j)
                if maze[i][j] == "G":
                    self.goal = Cell(i, j)
        print(self.start, self.goal)
        self.openSet = PriorityQueue()
        self.closedSet = set()
        self.cameFrom = {}
    
    def findShortestPath(self):
        start = self.start
        self.openSet.put(start, 0)
        self.closedSet.add(start)
        # current scores
        gScore = {}
        gScore[start] = 0
        # best guess at cost of path through n
        # fScore(n) = gScore(n) + h(n)
        ## fScore = {}
        ## fScore[start] = self.heuristic(start, self.goal)

        while not self.openSet.empty():
            # lowest at the top
            current = self.openSet.get()
            if current == self.goal:
                return self.reconstructPath(current)
            # print("current:", current)
            # print("visited", self.closedSet)
            for neighbor in self.getNeighbors(current):
                # print("queue", self.openSet.elements)
                tentativeGScore = gScore[current] + self.distance(current, neighbor)
                # print("n:", neighbor)
                # print("new start to n:", tentativeGScore, "known start to n:", gScore.get(neighbor, float("inf")))
                
                # if closer than currently known:
                if tentativeGScore < gScore.get(neighbor, float("inf")):
                    # print("update cameFrom")
                    self.cameFrom[neighbor] = current
                    gScore[neighbor] = tentativeGScore
                    # print("start-n-goal:", fScore[neighbor], "last leg", self.heuristic(neighbor, self.goal))
                    neighbor.f = tentativeGScore + self.heuristic(neighbor, self.goal)
                    if neighbor not in self.closedSet:
                        self.closedSet.add(neighbor)
                        self.openSet.put(neighbor, neighbor.f)
    
        return None
    
    def getNeighbors(self, current):
        r = []
        for dir in self.dirs:
            y = current.row + dir[1]
            x = current.col + dir[0]            
            if self.maze[y][x] != "#":
                r.append(Cell(y, x))
        return r

    def heuristic(self, start: Cell, goal: Cell):
        # manhattan distance
        return abs(start.row - goal.row) + abs(start.col - goal.col)
    
    def distance(self, current, neighbor):
        return 1.5e-4
    
    def reconstructPath(self, current) -> List[Cell]:
        total_path = [current]
        while current in self.cameFrom:
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path
    
    def print_path(self):
        path = self.reconstructPath(self.goal)
        print(path)
        maze = self.maze
        path_char = "|"
        for cell in path:
            if maze[cell.row][cell.col] not in ["S", "G"]:
                maze[cell.row][cell.col] = path_char
        for i in range(len(maze)):
            for j in range(len(maze[0])):
                if maze[i][j] not in ["#", "S", "G", path_char]:
                    print(' ', end='')
                else:
                    print(maze[i][j], end='')
            print()

if __name__ == "__main__":
    print("START")
    algo = AStar(maze)
    print("ALGO")
    algo.findShortestPath()
    algo.print_path()