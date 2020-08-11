# -*- encoding: utf-8 -*-
'''
@File    :   xiongyali.py
@Time    :   2020/08/11 10:41:13
@Author  :   Yan Hui 
@Version :   1.0
@Contact :   yanhui13@nudt.edu.cn
'''
from scipy.optimize import linear_sum_assignment
import numpy as np  

def construct_cost_matrix(delta_state, distance_matrix):
    """
    @description:
    构造优化问题cost矩阵
    @param:
    状态相对变化列表, 无向图节点之间最短路径矩阵
    @Returns:
    优化问题cost矩阵，输出点图id列表，输入点图id列表
    """
    cost_list = []
    out_id_list = []
    in_id_list = []
    for i in range(len(delta_state)):
        if delta_state[i] < 0:
            for j in range(abs(delta_state[i])):
                out_id_list.append(i)
        if delta_state[i] > 0:
            for k in range(delta_state[i]):
                in_id_list.append(i)
    for out_id in out_id_list:
        tmp_cost = []
        for in_id in in_id_list:
            tmp_cost.append(distance_matrix[out_id][in_id])
        cost_list.append(tmp_cost)
    return np.array(cost_list), out_id_list, in_id_list

def optimize(cost):
    """
    @description:
    匈牙利法找到最优分配方式
    @param:
    优化问题cost矩阵
    @Returns:
    输出点矩阵id列表，输入点id列表，分配成本列表，总体成本
    """
    out_id, in_id = linear_sum_assignment(cost)
    cost_index = cost[out_id, in_id]
    total_cost = round(cost[out_id, in_id].sum(), 1)
    return out_id, in_id, cost_index, total_cost


if __name__ == "__main__":
    cost = np.random.randint(1, 9, (4, 4))
    row_ind, col_ind, cost_ind, total_cost = optimize(cost)
    print(row_ind)#开销矩阵对应的行索引,对应于输出无人机的区域的列表指示器，需要与对应区域的ID映射
    print(col_ind)#对应行索引的最优指派的列索引，对应于输入无人机的区域的列表指示器，需要与对应区域的ID映射
    print(cost_ind)#提取每个行索引的最优指派列索引所在的元素的值索引，对应于无人机从输出点到输入点的飞行路径，需要与无人机绑定
    print(total_cost)#提取每个行索引的最优指派列索引所在的元素值的综合，表示本次规划无人机共需要飞行的距离
