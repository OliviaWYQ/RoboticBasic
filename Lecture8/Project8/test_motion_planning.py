from Lecture8.Project8.motion_planning import get_path_from_A_star
import matplotlib.pyplot as plt

if __name__ == '__main__':
    start = (0, 0)
    goal = (1, -5)
    obstacles = [(0, -2), (1, -2), (2, -2), (0, -3), (1, -3), (2, -3)]

    path = get_path_from_A_star(start, goal, obstacles)
    print("Path:", path)

    # ---- 可视化网格 ----
    _, ax = plt.subplots(figsize=(8, 6))
    ax.set_aspect('equal')

    # 收集所有点确定边界
    all_pts = [start, goal] + obstacles + path
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    margin = 1

    # 画网格线
    for x in range(min(xs) - margin, max(xs) + margin + 1):
        ax.axvline(x - 0.5, color='gray', alpha=0.3)
    for y in range(min(ys) - margin, max(ys) + margin + 1):
        ax.axhline(y - 0.5, color='gray', alpha=0.3)

    # 画障碍物
    ox, oy = zip(*obstacles) if obstacles else ([], [])
    ax.scatter(ox, oy, c='black', s=200, marker='s', label='Obstacles',
               zorder=3)

    # 画路径
    if path:
        px, py = zip(*path)
        ax.plot(px, py, 'b-', linewidth=2, label='Path', zorder=2)
        ax.scatter(px, py, c='blue', s=40, zorder=3)

    # 画起点和终点
    ax.scatter(*start, c='green', s=300, marker='o', label='Start',
               zorder=4, edgecolors='black')
    ax.scatter(*goal, c='red', s=300, marker='*', label='Goal',
               zorder=4, edgecolors='black')

    # 设置网格刻度
    ax.set_xticks(range(min(xs) - margin, max(xs) + margin + 1))
    ax.set_yticks(range(min(ys) - margin, max(ys) + margin + 1))
    ax.grid(True, alpha=0.15)
    ax.legend()
    ax.set_title('A* Motion Planning')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    # Y 轴反转让负值在下（和 PDF 截图一致）
    ax.invert_yaxis()
    plt.show()
