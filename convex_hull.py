import random
import matplotlib.pyplot as plt

# point class with x, y as point
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    

    def __str__(self):
        return '(%.2f, %.2f)' % (self.x, self.y)


def left_index(points):
    # Finding the left most point
    minn = 0
    for i in range(1,len(points)):
        if points[i].x < points[minn].x:
            minn = i
        elif points[i].x == points[minn].x:
            if points[i].y > points[minn].y:
                minn = i
    return minn


def orientation(p, q, r):
    # To find orientation of ordered triplet (p, q, r). The function returns following values:
    # 0 --> p, q and r are collinear
    # 1 --> Clockwise
    # 2 --> Counterclockwise
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    if val == 0:
        return 0
    elif val > 0:
        return 1
    else:
        return 2


def convexHull(points, n):
    # There must be at least 3 points
    if n < 3:
        return []
    # Find the leftmost point
    l = left_index(points)
    hull = []
    # Start from leftmost point, keep moving counterclockwise until reach the start point again. This loop runs O(h) times where h is number of points in result or output.
    p = l
    q = 0
    while(True):
        # Add current point to result
        hull.append(p)
        # Search for a point 'q' such that orientation(p, q, x) is counterclockwise for all points 'x'. The idea is to keep track of last visited most counterclockwise
        # point in q. If any point 'i' is more counterclockwise than q, then update q.

        q = (p + 1) % n

        for i in range(n):
            # If i is more counterclockwise than current q, then update q
            if(orientation(points[p], points[i], points[q]) == 2):
                q = i
        # Now q is the most counterclockwise with respect to p. Set p as q for next iteration, so that q is added to result 'hull'
        p = q
        # While we don't come to first point
        if(p == l):
            break
    return hull


points = []
maxPuntos = int(input("Numero de puntos a generar: "))

for i in range(maxPuntos):
    points.append(Point(random.randint(-100, 100), random.randint(-100, 100)))

hull = convexHull(points, len(points))
for idx in hull:
    print(points[idx])

#Generate chain for connection plot
chain = []
for idx_hull in range(len(hull)):
    #index of previous connection point
    p_sol = hull[(idx_hull - 1) % len(hull)]
    #index of current connection point
    sol = hull[idx_hull % len(hull)]
    #This is what is going to be given to the plotter (two points connected)
    chain.append([
        [points[p_sol].x, points[sol].x], #X's
        [points[p_sol].y, points[sol].y], #Y's
    ])

scatter_x = [p.x for p in points]
scatter_y = [p.y for p in points]

plt.scatter(scatter_x, scatter_y)
for connection in chain:
    xs = connection[0]
    ys = connection[1]
    plt.plot(xs, ys, 'r-')
plt.show()
