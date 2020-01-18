from Node import State, Node

class OccupancyGrid():
    def __init__(self, map):
        self.grid = []
        for i in range(len(map)):
            row = []
            for j in range(len(map[0])):
                if map[i][j] == 1:
                    # print('*', end=' ')
                    row.append(Node(State.FULL))
                if map[i][j] == 0:
                    # print(' ', end=' ')
                    row.append(Node(State.EMPTY))
            self.grid.append(row)
    
    def SetGoal(self, x, y):
        # calculate node hn here
        pass

    def SetStart(self, x, y):
        # calculate node gn here
        pass
    
    def Print(self):
        print("printing grid...")
        for i in range(len(self.grid)):
            for j in range(len(self.grid[0])):
                if self.grid[i][j].state == State.FULL:
                    print('*', end=' ')
                if self.grid[i][j].state == State.EMPTY:
                    print(' ', end=' ')
            print()

