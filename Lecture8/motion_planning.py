"""实验 8：使用 A* 算法完成四连通网格上的运动规划。"""

from heapq import heappop, heappush
from itertools import count


# 四连通网格中只允许上、下、左、右移动，不允许沿对角线移动。
_DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))


def _manhattan_distance(point, goal):
    """返回两点的曼哈顿距离，它是四连通网格上的可采纳启发函数。"""
    return abs(point[0] - goal[0]) + abs(point[1] - goal[1])


def _reconstruct_path(came_from, start, goal):
    """从父节点字典反向恢复路径，并按实验要求去掉起点。"""
    path = []
    current = goal

    # goal、goal 的父节点……依次加入列表，直到回到 start。
    while current != start:
        path.append(current)
        current = came_from[current]

    path.reverse()
    return path


def get_path_from_A_star(start, goal, obstacles):
    """计算从 ``start`` 到 ``goal`` 的一条最短无碰撞路径。

    参数：
        start: 起始网格，形如 ``(x, y)`` 的整数二元组。
        goal: 目标网格，形如 ``(x, y)`` 的整数二元组。
        obstacles: 障碍物网格组成的可迭代对象。

    返回：
        路径二元组列表。列表包含目标、不包含起点；若不存在路径则返回
        空列表。起点与目标相同时也返回空列表，因为机器人无需移动。

    算法说明：
        A* 按 f(n) = g(n) + h(n) 选择下一个扩展节点。其中 g 是从
        起点走到当前节点的实际步数，h 使用曼哈顿距离。曼哈顿距离不会
        高估四连通网格中的剩余步数，因此 A* 找到目标时得到的是最短路。
    """
    # 统一转换为元组，便于把网格点用作集合成员和字典键。
    start = tuple(start)
    goal = tuple(goal)
    obstacle_set = {tuple(obstacle) for obstacle in obstacles}

    # 目标（或起点）位于障碍物中时，不存在合法路径。
    if start in obstacle_set or goal in obstacle_set:
        return []
    if start == goal:
        return []

    # 地图在题目中没有给定边界。为了让“目标被完全包围”的情况也能
    # 正常结束，将搜索限制在所有相关网格的外一圈。所有障碍物都在此
    # 矩形内部，而外圈没有障碍；最短路无需绕到更远的区域。
    relevant_points = obstacle_set | {start, goal}
    min_x = min(point[0] for point in relevant_points) - 1
    max_x = max(point[0] for point in relevant_points) + 1
    min_y = min(point[1] for point in relevant_points) - 1
    max_y = max(point[1] for point in relevant_points) + 1

    # open_heap 保存 (f, h, 插入次序, 节点)。h 和插入次序只用于在 f
    # 相同时稳定地打破平局，避免直接依赖节点坐标的比较顺序。
    open_heap = []
    insertion_order = count()
    start_h = _manhattan_distance(start, goal)
    heappush(open_heap, (start_h, start_h, next(insertion_order), start))

    # g_score[n] 是目前找到的从 start 到 n 的最小代价。
    g_score = {start: 0}
    # came_from[n] 记录最优路径上 n 的前一个节点，用于最后恢复路径。
    came_from = {}

    while open_heap:
        queued_f, _, _, current = heappop(open_heap)
        current_g = g_score[current]

        # 同一节点在发现更短路径后可能被再次压入堆。旧条目的 f 值较大，
        # 弹出时直接忽略即可，不需要在 heapq 中做昂贵的删除操作。
        expected_f = current_g + _manhattan_distance(current, goal)
        if queued_f != expected_f:
            continue

        if current == goal:
            return _reconstruct_path(came_from, start, goal)

        for dx, dy in _DIRECTIONS:
            neighbor = (current[0] + dx, current[1] + dy)

            # 跳过搜索边界以外的节点和障碍物节点。
            if not (min_x <= neighbor[0] <= max_x and min_y <= neighbor[1] <= max_y):
                continue
            if neighbor in obstacle_set:
                continue

            # 每次水平或竖直移动一格，边的代价恒为 1。
            tentative_g = current_g + 1
            if tentative_g >= g_score.get(neighbor, float("inf")):
                continue

            # 找到了到 neighbor 的更短路径，更新父节点和代价并加入 open。
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            neighbor_h = _manhattan_distance(neighbor, goal)
            heappush(
                open_heap,
                (
                    tentative_g + neighbor_h,
                    neighbor_h,
                    next(insertion_order),
                    neighbor,
                ),
            )

    # open 集合耗尽仍未到达目标，说明目标与起点不连通。
    return []
