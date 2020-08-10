from scipy.optimize import linear_sum_assignment
import numpy as np  # 使用import导入模块numpy，并简写成np
cost =np.array([[3,1,2],[4,3,4],[2,5,1]])
def optimize(cost):
    row_ind, col_ind=linear_sum_assignment(cost)
    cost_ind = cost[row_ind,col_ind]
    total_cost = cost[row_ind,col_ind].sum()
    return row_ind, col_ind, cost_ind, total_cost

row_ind, col_ind, cost_ind, total_cost = optimize(cost)
print(row_ind)#开销矩阵对应的行索引,对应于输出无人机的区域的列表指示器，需要与对应区域的ID映射
print(col_ind)#对应行索引的最优指派的列索引，对应于输入无人机的区域的列表指示器，需要与对应区域的ID映射
print(cost_ind)#提取每个行索引的最优指派列索引所在的元素的值索引，对应于无人机从输出点到输入点的飞行路径，需要与无人机绑定
print(total_cost)#提取每个行索引的最优指派列索引所在的元素值的综合，表示本次规划无人机共需要飞行的距离
