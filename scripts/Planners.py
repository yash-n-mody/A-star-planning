from __future__ import print_function
import Map
import math

class BaseGlobalPlanner():
    def __init__(self):
        self.tolerance = 0.25
        pass

    def SetPath(self, path):
        self.base = path
        self.plan = self.DecodePath(path)
        self.pointer = 0

    def PrintPlan(self):
        for i in range(len(Map.grid)):
            for j in range(len(Map.grid[i])):
                if Map.grid[i][j] == 1:
                    print('#', end=' ')
                elif (i, j) in self.base:
                    print(".", end=' ')
                else:
                    print(' ', end=' ')
            print(end='\n')

    def DecodePath(self, path):
        plan = []
        for t in path:
            point = {}
            x, y = Map.ToWorldCoord(t[1], t[0])
            point['x'] = x
            point['y'] = y
            plan.append(point)
        return plan

    def GetDirections(self, curr_x, curr_y, curr_theta):
        if self.HasPlan():
            next_p = self.plan[self.pointer]
            r = self.DeltaTranslation(curr_x, curr_y, next_p['x'], next_p['y'])
            theta = self.DeltaRotation(curr_x, curr_y, curr_theta, next_p['x'], next_p['y'])
            return r, theta
        else:
            return 0, 0

    def DeltaRotation(self, sx, sy, stheta, tx, ty):
        ttheta = math.atan2(ty - sy, tx - sx) * 180/3.14
        # map [-pi, pi] to [0, 2pi]
        if ttheta < 0:
            ttheta = 360 + ttheta
        diff = ttheta - stheta
        # map [0,2pi] to [-pi, pi]
        if diff > 180:
            diff = diff - 360
        return diff

    def DeltaTranslation(self, sx, sy, tx, ty):
        return math.sqrt(math.pow((sx - tx), 2) + math.pow((sy - ty), 2))

    def HasPlan(self):
        if (len(self.plan) - self.pointer) > 0:
            return True
        else:
            return False

    def GetGoal(self, curr_x, curr_y):
        g_x = self.plan[self.pointer]['x']
        g_y = self.plan[self.pointer]['y']
        there_x = False
        there_y = False
        if g_x - self.tolerance <= curr_x <= g_x + self.tolerance:
            there_x = True
        if g_y - self.tolerance <= curr_y <= g_y + self.tolerance:
            there_y = True
        if there_x and there_y:
            self.pointer = self.pointer + 1
        if self.HasPlan():    
            return self.plan[self.pointer]['x'], self.plan[self.pointer]['y']
        else:
            index = len(self.plan) - 1
            return self.plan[index]['x'], self.plan[index]['y']
