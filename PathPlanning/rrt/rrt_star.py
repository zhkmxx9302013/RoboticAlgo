import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt

show_animation = True


class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList, randArea,
                 expandDis=0.5, goalSampleRate=20, maxIter=500):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True, search_until_maxiter=True):
        """
        rrt path planning
        animation: flag for animation on or off
        search_until_maxiter: search until max iteration for path improving or not
        """

        self.nodeList = [self.start]
        for i in range(self.maxIter):
            rnd = self.get_random_point()                           # 随机选点
            nind = self.GetNearestListIndex(self.nodeList, rnd)     # 选择树中与随机点最近的点

            new_node = self.steer(rnd, nind)                        # 从离随机点最近的点到随机点的方向上迈出一步固定步长

            if self.__CollisionCheck(new_node, self.obstacleList):  # 避障检测
                nearinds = self.find_near_nodes(new_node)           # 在随机选点的某个半径之内选择树上的点
                new_node = self.choose_parent(new_node, nearinds)   # 选择新节点的父节点，选一个cost最小的，cost是从头到当前加入节点的路径长度和，树的每个节点都有一个当前的cost值
                self.nodeList.append(new_node)
                self.rewire(new_node, nearinds)                     # 选择新点后，进行重新构造父节点，对树的结构进行调整

            if animation and i % 5 == 0:
                self.DrawGraph(rnd, nearinds)

            # generate course
            if not search_until_maxiter:
                lastIndex = self.get_best_last_index()
                if lastIndex:
                    return self.gen_final_course(lastIndex)

        print("reached max iteration")

        lastIndex = self.get_best_last_index()
        if lastIndex:
            return self.gen_final_course(lastIndex)

        return None

    def choose_parent(self, new_node, nearinds):
        """
        选择新节点的父节点，选一个cost最小的，cost是从头到当前加入节点的路径长度和，树的每个节点都有一个当前的cost值
        :param new_node:
        :param nearinds:
        :return:
        """
        if not nearinds:
            return new_node

        dlist = []
        for i in nearinds:
            dx = new_node.x - self.nodeList[i].x
            dy = new_node.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return new_node

        new_node.cost = mincost
        new_node.parent = minind

        return new_node

    def steer(self, rnd, nind):
        """
        生成新的树节点，扩展树，向前走一步
        :param rnd:
        :param nind:
        :return:
        """
        # expand tree
        nearest_node = self.nodeList[nind]
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node = Node(rnd[0], rnd[1])
        currentDistance = math.sqrt(
            (rnd[1] - nearest_node.y) ** 2 + (rnd[0] - nearest_node.x) ** 2)
        # Find a point within expandDis of nind, and closest to rnd
        if currentDistance <= self.expandDis:
            pass
        else:
            new_node.x = nearest_node.x + self.expandDis * math.cos(theta)
            new_node.y = nearest_node.y + self.expandDis * math.sin(theta)
        new_node.cost = float("inf")
        new_node.parent = None
        return new_node

    def get_random_point(self):
        """
        生成随机点
        """
        if random.randint(0, 100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]

        return rnd

    def get_best_last_index(self):

        disglist = [self.calc_dist_to_goal(
            node.x, node.y) for node in self.nodeList]
        goalinds = [disglist.index(i) for i in disglist if i <= self.expandDis]

        if not goalinds:
            return None

        mincost = min([self.nodeList[i].cost for i in goalinds])
        for i in goalinds:
            if self.nodeList[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.nodeList[goalind].parent is not None:
            node = self.nodeList[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, new_node):
        """
        在新随机选出的点周围的半径范围内，选择树上的点构建Near集合
        :param new_node:
        :return:
        """
        nnode = len(self.nodeList)
        r = 10.0 * math.sqrt((math.log(nnode) / nnode))
        dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]

        return nearinds

    def rewire(self, new_node, nearinds):
        """
        选择新点后，进行重新构造父节点，对树的结构进行调整, 把新点加入树后，原来的Near集中的点的cost可能不如通过新加入的点这么绕一下
        :param new_node:
        :param nearinds:
        :return:
        """
        nnode = len(self.nodeList)
        for near_idx in nearinds:  # 遍历所有Near集中的节点
            nearNode = self.nodeList[near_idx]

            dx = new_node.x - nearNode.x
            dy = new_node.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = new_node.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):
        """
        父节点与新节点之间的连线是否能够避撞
        :param nearNode:
        :param theta:
        :param d:
        :return:
        """
        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):#多步迭代，判断父节点与新节点之间的连线是否能够避撞
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False
        # tmpNode.x += d * math.cos(theta)
        # tmpNode.y += d * math.sin(theta)
        # if not self.__CollisionCheck(tmpNode, self.obstacleList):
        #     return False
        return True

    def DrawGraph(self, rnd=None, nearinds=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=30 * size)

        if nearinds is not None:
            near_x = []
            near_y = []
            for i in nearinds:
                near_x.append(self.nodeList[i].x)
                near_y.append(self.nodeList[i].y)
            plt.scatter(near_x, near_y, color='r')

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))

        return minind

    def __CollisionCheck(self, node, obstacleList):
        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision

        return True  # safe


def main():
    print("Start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[10, 10],
              randArea=[-2, 15], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation, search_until_maxiter=False)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.DrawGraph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)
            plt.show()


if __name__ == '__main__':
    main()