# -*-coding:utf-8 -*-
import random
import math
import numpy as np
import matplotlib.pyplot as plt

#初始化生成chromosome_length大小的population_size个个体的二进制基因型种群
def species_origin(population_size,chromosome_length):
    # 二维列表，包含染色体和基因
    population=[]
    # 生成特定规模的种群
    for i in range(population_size):
        # 染色体暂存器
        chromosome=[]
        # 生成特定长度的染色体
        for j in range(chromosome_length):
            # 随机产生一个染色体,由二进制数组成
            chromosome.append(random.randint(0,1))
        population.append(chromosome)
            # 将种群返回，种群是个二维数组，个体和染色体两维
    return population


 #input:种群,染色体长度, 变量实际范围
def population_translation(population,chromosome_length, var_range):
    translation_var = []
    for i in range(len(population)):
        chromosome = np.array(population[i])
        # 如：01010 转成十进制为：1*2^3+1*2^1 = 10
        var = chromosome_translation(chromosome, chromosome_length, var_range)
        translation_var.append(var)
    # 返回种群中所有个体编码完成后的十进制数
    return translation_var

#从二进制到十进制染色体翻译
def chromosome_translation(chromosome, chromosome_length, var_range):
    chromosome_total = chromosome.dot(2 ** np.arange(chromosome_length)[::-1])
    translation_upper = math.pow(2, chromosome_length) - 1
    var = chromosome_total / translation_upper * (var_range[1] - var_range[0]) + var_range[0]
    return var

# 定义目标函数
def function(x):
    func = 2 * math.sin(x) + math.cos(x)
    # func = -math.pow(x,3) + 10* math.pow(x,2) + math.sqrt(x)
    return func

# 绘制目标函数图像
def draw_function(x_range):
    x_list = (np.arange(x_range[0], x_range[1]*10) / 10).tolist()
    y_list = []
    for x in x_list:
        y = function(x)
        y_list.append(y)
    plt.plot(x_list, y_list)
    plt.xlim(x_range[0], x_range[1])
    if min(y_list) > 0:
        y_min = min(y_list) * 0.8
    else:
        y_min = min(y_list) * 1.2
    if max(y_list) > 0:
        y_max = max(y_list) * 1.2
    else:
        y_max = max(y_list) * 0.8
    plt.ylim(y_min, y_max)
    plt.show()
    y_range = [y_min, y_max]
    return y_range

# 目标函数相当于环境 对染色体进行筛选，这里是2*sin(x)+cos(x)
def get_cost_function(translation_var):
    cost_func = []
    for i in range(len(translation_var)):
        # 这里将2*sin(x)+cos(x)作为目标函数，也是适应度函数
        x = translation_var[i]
        func = function(x)
        cost_func.append(func)
    return cost_func

# 计算目标最小化（True）或最大化（False）情况下的适应度：根据目标函数输出计算染色体的适应度
def get_fitness(func, optimal_type):
    fit = []
    punish_factor = 0.1
    min_func = min(func)  # 寻找输出最小值
    max_func = max(func)  # 寻找输出最大值
    if optimal_type is True:
        # 目标最小化优化问题
        for i in range(len(func)):
            fit_temp = -1 * (func[i] - max_func) + 0.0001  # 0.0001为了防止适应度为0，无法赋予概率值
            if min_func - func[i] < min_func: # 消除差值抵消导致的劣解概率提高的问题
                fit_temp = fit_temp * punish_factor
            fit.append(fit_temp)
    else:
        for i in range(len(func)):
            fit_temp = func[i] - min_func + 0.0001  # 0.0001为了防止适应度为0，无法赋予概率值
            if max_func - func[i] > max_func: # 消除差值抵消导致的劣解概率提高的问题
                fit_temp = fit_temp * punish_factor
            fit.append(fit_temp)
    return fit

# 根据染色体的适应度挑选合适的种群
def species_selection(population, population_size, fitness):
    selected_population = []
    np_fitness = np.array(fitness) # 列表转np运算，方便向量除法运算
    probability = (np_fitness / np_fitness.sum()).tolist()
    #print('probability: ', probability)
    # np.random.choice：挑选出满足概率分布p，规模为size的id集合
    idx = np.random.choice(np.arange(population_size), size = population_size, replace = True, p = probability)
    #print('select species id ', idx)
    for id in idx:
        selected_population.append(population[id])
    return selected_population

#　染色体交叉
def crossover(population, population_size, chromosome_length, crossover_rate):
    new_population = []
    for father in population: # 遍历种群中的每一个个体，将该个体作为父亲
        child = father # 孩子先得到父亲的全部基因
        if np.random.rand() < crossover_rate: # 产生子代时不是必然发生交叉，而是以一定的概率发生交叉
            #print('**********发生了交叉**********')
            mother = population[np.random.randint(population_size)]	# 在种群中选择另一个个体，并将该个体作为母亲
            cross_point = np.random.randint(low = 0, high=chromosome_length) # 随机产生交叉的点
            child[cross_point:] = mother[cross_point:] # 孩子得到位于交叉点后的母亲的基因
            #print(cross_point, father, mother, child)
        new_population.append(child)
    return new_population

# 染色体上某个基因变异
def mutation(population, chromosome_length, mutation_rate):
    for pop in population:
        if np.random.rand() < mutation_rate: # 以MUTATION_RATE的概率进行变异
            #print('**********发生了变异**********')
            mutate_point = np.random.randint(low = 0, high=chromosome_length)	#随机产生变异点
            pop[mutate_point] = pop[mutate_point]^1 	#将变异点的二进制反转
    return population

# 获得每轮得到的最优结果
def get_optimal_result(population, func, translation_result, optimal_type):
    if optimal_type is True:
        optimal_func = min(func)
    else:
        optimal_func = max(func)
    optimal_func_index = func.index(optimal_func)
    optimal_population = population[optimal_func_index]
    optimal_var = translation_result[optimal_func_index]
    return optimal_func, optimal_var, optimal_population

# 输出最优的目标值，变量值，染色体值，同时绘制排序后的最优目标值趋势
def result_show(optimal_func_list, optimal_var_list, optimal_pop_list, optimal_type, y_range):
    if optimal_type is True:
        best_func = min(optimal_func_list)
    else:
        best_func = max(optimal_func_list)
    best_func_index = optimal_func_list.index(best_func)
    best_var = optimal_var_list[best_func_index]
    best_pop = optimal_pop_list[best_func_index]
    print('******************* optimal result *********************')
    print('optimal_func       ', best_func)
    print('optimal_var        ', best_var)
    print('optimal_population ', best_pop)
    trend_show(optimal_func_list, optimal_type, y_range)

def trend_show(optimal_func_list, optimal_type, y_range):
    optimal_func_list.sort(reverse = optimal_type)
    func_num = len(optimal_func_list)
    X = list(range(func_num))
    plt.plot(X, optimal_func_list)
    plt.ylim(y_range[0], y_range[1])
    plt.show()

if __name__ == "__main__":
    population_size, chromosome_length = 200, 10
    crossover_rate, mutation_rate = 0.6, 0.01
    var_range = [0, 10]
    iteration_num = 50
    optimal_type = False # 优化类型是最大化
    optimal_func_list = []
    optimal_var_list = []
    optimal_pop_list = []
    y_range = draw_function(var_range)
    population = species_origin(population_size, chromosome_length)
    #print('population', len(population), population)
    for i in range(iteration_num):
        #print('*********************** ', i, ' ***********************')
        translation_result = chromosome_translation(population, chromosome_length, var_range)
        #print('translation', len(translation_result), translation_result)
        func = get_cost_function(translation_result)
        #print('function', len(func), func)
        fitness = get_fitness(func, optimal_type)
        #print('fitness_max', len(fitness), fitness)
        optimal_func, optimal_var, optimal_pop = get_optimal_result(population, func, translation_result, optimal_type)
        optimal_func_list.append(optimal_func)
        optimal_var_list.append(optimal_var)
        optimal_pop_list.append(optimal_pop)
        select_population = species_selection(population, population_size, fitness)
        #print('species_select', len(select_population), select_population)
        if i < iteration_num - 1:
            population = crossover(select_population, population_size, chromosome_length, crossover_rate)
            population = mutation(population, chromosome_length, mutation_rate)
        #print('population', len(population), population)
    result_show(optimal_func_list, optimal_var_list, optimal_pop_list, optimal_type, y_range)