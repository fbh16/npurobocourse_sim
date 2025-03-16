import numpy as np

# 圆心和半径
x_c, y_c = 3.6, -2.0
radius = 1

# 设置角度，均匀分布
theta = np.linspace(0, 2 * np.pi, num=10)  # 100 个点

# 计算轨迹点
x = x_c + radius * np.cos(theta)
y = y_c + radius * np.sin(theta)

# 打印坐标数列
trajectory = list(zip(x, y))
print('x: ', end='')
for point in trajectory:
    print(f"{point[0]:.3f}", end=', ')

print('\ny: ', end='')
for point in trajectory:
    print(f"{point[1]:.3f}", end=', ')
print('\n')
