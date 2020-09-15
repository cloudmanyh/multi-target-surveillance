# -*- encoding: utf-8 -*-
'''
@File        :   expSetting.py
@Description :   实验设置
@Time        :   2020/08/13 12:15:46
@Author      :   Yan Hui 
@Version     :   1.0
@Contact     :   yanhui13@nudt.edu.cn
'''
import pandas as pd
import numpy as np  # 使用import导入模块numpy，并简写成np
import random
import uuid
import os
import shutil
from paraTags import paraTags
import graph
import networkx as nx

def fileDirSetting(filepath):
    '''
    如果文件夹不存在就创建，如果文件存在就清空！
    :param filepath:需要创建的文件夹路径
    :return:
    '''
    if not os.path.exists(filepath):
        os.mkdir(filepath)
    else:
        shutil.rmtree(filepath)
        os.mkdir(filepath)

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
            if type(value)== list:
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

# 将list变量存为CSV文件
def writeList2CSV(myList,filePath):
    writer = pd.DataFrame(data=myList)
    writer.to_csv(filePath, encoding='gbk')

# 产生无人车的优先级列表，规则：数字下界为Lower，上界为Upper(不包含)，和为Sum，规模为Size
def generateStateList(Lower, Upper, Sum, Size):
    var = 1
    targetList=[]
    while var == 1:  # 该条件永远为true，循环将无限执行下去
        targetList = np.random.randint(Lower, Upper, size=Size)
        if sum(targetList) == Sum:
            break
    return targetList

#随机产生不同无人机Id对应不同颜色编码的颜色词典
def generateColorDict():
    colorDict = {}
    for k in range(paraTags.uavNum):
        ID = str(k)
        res = str(uuid.uuid4())
        res = res.replace('-', '')
        color = '#' + res[:6]
        colorDict[ID] = color
    return colorDict

# 随机产生不同无人机Id对应不同颜色编码的颜色词典
def Dict2List(dict):
    colorList = []
    keyList = list(dict.keys())
    valueList = list(dict.values())
    colorList.append(keyList)
    colorList.append(valueList)
    return colorList

# 将列表内容转为词典，列表必须为二维列表
def List2Dict(colorList):
    keyList = colorList[0]
    valueList = colorList[1]
    colorDict = dict(zip(keyList, valueList))
    return colorDict

# 根据仿真次数生成无人车的优先级状态信息列表
def ugvParaSetting():
    stateList = list()
    for i in range(paraTags.simNum + 1):
        state = generateStateList(paraTags.priorityLower, paraTags.priorityUpper, paraTags.uavNum, paraTags.ugvNum)
        stateList.append(state)
    writeList2CSV(stateList, paraTags.statePath)
    stateResult = readCSV2List(paraTags.statePath)
    print(stateResult)

# 根据无人机数量，生成无人机初始电量信息列表
def uavParaSetting():
    uavParaList = list()
    for k in range(paraTags.uavNum):
        uavState = []
        # uavState.append(paraTags.uavEnergy2)
        # uavState.append(paraTags.uavEnergyPower2)
        # uavState.append(paraTags.uavSpeed2)
        uavType = random.randint(1, 3)
        if uavType == 1:
            uavState.append(paraTags.uavEnergy1)
            uavState.append(paraTags.uavEnergyPower1)
            uavState.append(paraTags.uavSpeed1)
        elif uavType == 2:
            uavState.append(paraTags.uavEnergy2)
            uavState.append(paraTags.uavEnergyPower2)
            uavState.append(paraTags.uavSpeed2)
        else:
            uavState.append(paraTags.uavEnergy3)
            uavState.append(paraTags.uavEnergyPower3)
            uavState.append(paraTags.uavSpeed3)
        uavParaList.append(uavState)
    writeList2CSV(uavParaList, paraTags.energyPath)
    energyResult = readCSV2List(paraTags.energyPath)
    print(energyResult)

# 根据无人机数量，生成无人机画图对应颜色词典
def colorDictSetting():
    colorDict = generateColorDict()
    colorList = Dict2List(colorDict)
    writeList2CSV(colorList, paraTags.colorPath)
    colorResult = readCSV2List(paraTags.colorPath)
    colorDict = List2Dict(colorResult)
    print(colorDict)
# 根据目标数量和目标间连接度生成目标无向图
def graph_setting():
    G = nx.Graph()
    topology, node_labels = graph.generate_topology(paraTags.ugvNum, paraTags.connectivity)
    graph.build_graph(topology, G)
    edge_labels = nx.get_edge_attributes(G,'weight') # 生成连通边标签
    length_list, track_list = graph.get_graph_attribute(G)
    writeList2CSV(length_list, paraTags.graphLengthPath)
    writeList2CSV(track_list, paraTags.graphTrackPath)
    length_result = readCSV2List(paraTags.graphLengthPath)
    track_result = readCSV2List(paraTags.graphTrackPath)
    print('节点间距离矩阵：\n',length_result)
    print('最短间距路径：\n',track_result)
    graph.draw_graph(G, node_labels, edge_labels)

# # 根据目标数量和目标间连接度生成目标无向图
# def graph_setting():
#     length_list=[
#         [0.0,1.0,0.3,0.5,0.7], 
#         [1.0,0.0,0.7,0.6,0.5], 
#         [0.3,0.7,0.0,0.8,0.4], 
#         [0.5,0.6,0.8,0.0,0.9], 
#         [0.7,0.5,0.4,0.9,0.0]]
#     writeList2CSV(length_list, paraTags.graphLengthPath)
#     length_result = readCSV2List(paraTags.graphLengthPath)
#     print('节点间距离矩阵：\n',length_result)


if __name__ == "__main__":
    # 首先根据设定的存储路径生成文件夹，否则会报错
    filepath = paraTags.statePath.split('/', 1)[0]
    fileDirSetting(filepath)
    # 根据仿真次数生成无人车的优先级状态信息列表
    ugvParaSetting()
    # 根据无人机数量，生成无人机初始电量信息列表
    uavParaSetting()
    # 根据无人机数量，生成无人机画图对应颜色词典
    colorDictSetting()
    # 生成节点图属性
    graph_setting()
    
