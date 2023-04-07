"""
Dijkstra 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq # 堆队列（heap queue）

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

from Search_2D import plotting, env

from Search_2D.Astar import AStar


class Dijkstra(AStar):
    """Dijkstra set the cost as the priority 
    """
    def searching(self):
        """
        Breadth-first Searching. 广度优先搜索
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start # 将起点的父节点设置为它本身
        self.g[self.s_start] = 0 # 将起点到起点的成本设置为0
        self.g[self.s_goal] = math.inf # 将终点到起点的成本设置为无穷大
        heapq.heappush(self.OPEN, # 堆队列（heap queue），将起点加入到OPEN列表中
                       (0, self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal: # 找到目标，跳出循环
                break

            for s_n in self.get_neighbor(s): # 遍历相邻节点
                new_cost = self.g[s] + self.cost(s, s_n) # 计算新的成本

                if s_n not in self.g: # 如果节点没有被探索过，将其成本设为无穷大
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # 满足更新成本的条件
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s

                    # 最佳优先级使用启发式作为优先级
                    heapq.heappush(self.OPEN, (new_cost, s_n))

        return self.extract_path(self.PARENT), self.CLOSED


def main():
    s_start = (5, 5) # 设置起点
    s_goal = (45, 25) # 设置终点

    dijkstra = Dijkstra(s_start, s_goal, 'None') # 创建Dijkstra对象
    plot = plotting.Plotting(s_start, s_goal) # 创建绘图对象

    path, visited = dijkstra.searching() # 运行Dijkstra算法
    plot.animation(path, visited, "Dijkstra's")  # 生成动画

if __name__ == '__main__':
    main()
