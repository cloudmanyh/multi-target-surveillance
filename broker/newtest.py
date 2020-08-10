import graph
import xiongyali as XYL
import numpy as np  # 使用import导入模块numpy，并简写成np
import pandas as pd
from paraTags import paraTags

# 将混合列表转化为一维列表
def flatten(input_list):
    output_list = []
    while True:
        if input_list == []:
            break
        for index, value in enumerate(input_list):
            # index :索引序列  value:索引序列对应的值
            # enumerate() 函数用于将一个可遍历的数据对象(如列表、元组或字符串)组合为一个索引序列，
            # 同时列出数据和数据下标，一般用在 for 循环当中。
            if type(value) == list:
                input_list = value + input_list[index+1:]
                break   # 这里跳出for循环后，从While循环进入的时候index是更新后的input_list新开始算的。
            else:
                output_list.append(value)
                input_list.pop(index)
                break
    return output_list

# 读入CSV文件并存为list类型
def readCSV2List(filePath):
    corpus = pd.read_csv(filePath)
    result = corpus.values.tolist()
    for k in range(len(result)):
        tempList = result[k]
        del tempList[0]
    if len(result[0]) == 1:
        # 说明二维列表内部列表只有一个变量，本质上是一维列表，压缩为一维列表
        result = flatten(result)
    return result


if __name__ == "__main__":
    distance_matrix = np.array(readCSV2List(paraTags.graphLengthPath))
    print('原始距离矩阵：\n', distance_matrix)
    delta_state = [2, -3, -1, 3, -1]
    cost_matrix, out_id_list, in_id_list = XYL.construct_cost_matrix(
        delta_state, distance_matrix)
    print('out_id_list: ', out_id_list)
    print('in_id_list: ', in_id_list)
    print('优化目标矩阵：\n', cost_matrix)
    out_id, in_id, cost_index, total_cost = XYL.optimize(cost_matrix)
    print('输出方id：', out_id)  # 开销矩阵对应的行索引,对应于输出无人机的区域的列表指示器，需要与对应区域的ID映射
    print('输入方id：', in_id)  # 对应行索引的最优指派的列索引，对应于输入无人机的区域的列表指示器，需要与对应区域的ID映射
    print('对应cost：', cost_index)  # 提取每个行索引的最优指派列索引所在的元素的值索引，对应于无人机从输出点到输入点的飞行路径，需要与无人机绑定
    print('总体cost：', total_cost)  # 提取每个行索引的最优指派列索引所在的元素值的综合，表示本次规划无人机共需要飞行的距离
    system_strategy = []
    for i in range(len(out_id)):
        former = out_id_list[out_id[i]]
        current = in_id_list[in_id[i]]
        cost = cost_index[i]
        system_strategy.append([former, current, cost])
    print(system_strategy)
