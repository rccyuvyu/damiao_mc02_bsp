# 读取csv文件，其第一行是标题，计算每一列数据的均值和标准差

import numpy as np
import pandas as pd

def compute_mean_std(file_path):
    # 读取csv文件
    data = pd.read_csv(file_path)
    
    # 计算每一列的均值和标准差
    means = data.mean()
    stds = data.std()
    
    return means, stds

if __name__ == "__main__":
    file_path = './gyro_calib.csv'  # 替换为你的csv文件路径
    means, stds = compute_mean_std(file_path)

    # 打印输出并保留八位小数
    print(f"Means:{means.round(8).to_dict()}")
    print(f"Stds:{stds.round(8).to_dict()}")