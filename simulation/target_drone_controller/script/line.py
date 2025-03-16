import numpy as np

def generate_arithmetic_sequence_2d(start, end, num_points):
    """
    生成从起点到终点的二维等差数列

    参数:
    start (tuple): 起点坐标 (x1, y1)
    end (tuple): 终点坐标 (x2, y2)
    num_points (int): 数列中点的数量

    返回:
    tuple: 生成的二维等差数列的 x 和 y 坐标
    """
    x1, y1 = start
    x2, y2 = end

    # 对每个坐标分量生成等差数列
    x_sequence = np.linspace(x1, x2, num_points)
    y_sequence = np.linspace(y1, y2, num_points)

    return x_sequence, y_sequence

def main():
    # 定义起点坐标、终点坐标和数列长度
    start_point = (4.5, -10.0)  # 起点 (x1, y1)
    end_point = (-2.32, 4.28)    # 终点 (x2, y2)
    num_points = 8        # 数列长度
    
    # 生成二维等差数列的 x 和 y 坐标
    x_sequence, y_sequence = generate_arithmetic_sequence_2d(start_point, end_point, num_points)
    
    # 输出 x 和 y 坐标，每个坐标保留三位小数，且以逗号隔开
    print("x坐标: ")
    print(", ".join([f"{x:.3f}" for x in x_sequence]))
    
    print("y坐标: ")
    print(", ".join([f"{y:.3f}" for y in y_sequence]))

if __name__ == "__main__":
    main()
# 02   （4.5，-10.0）；（ -2.32，4.28）      01 （0.5，-10.0）（ 6.60， 4.54）