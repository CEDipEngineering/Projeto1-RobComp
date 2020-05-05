class Point():

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def getTuple(self):
        return (self.x, self.y)



class Line():

    def __init__(self, point1, point2):
        self.point1 = point1
        self.point2 = point2
        if self.point1.x == self.point2.x:
            self.m = 100000
        else:
            self.m = (point1.y-point2.y)/(point1.x-point2.x)

    def intersect(self, line):
        x1 = self.point1.x
        x2 = self.point2.x
        x3 = line.point1.x        
        x4 = line.point2.x
        y1 = self.point1.y
        y2 = self.point2.y
        y3 = line.point1.y
        y4 = line.point2.y
        out_x = int(round(((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4))/((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))))
        out_y = int(round(((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4))/((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))))
        out = Point(out_x, out_y)
        return out
    
    def getPoints(self):
        return ((self.point1.x, self.point1.y), (self.point2.x, self.point2.y))
