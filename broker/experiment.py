# -*- encoding: utf-8 -*-
'''
@File        :   experiment.py
@Description :   实验入口
@Time        :   2020/08/13 12:15:17
@Author      :   Yan Hui 
@Version     :   1.0
@Contact     :   yanhui13@nudt.edu.cn
'''
import numpy as np  # 使用import导入模块numpy，并简写成np
import matplotlib.pyplot as plt  # 使用import导入模块matplotlib.pyplot，并简写成plt
import pandas as pd
import os
import shutil
from UAV import UAV
from UGV import UGV
from paraTags import paraTags
from Broker import Broker

plt.rcParams['font.sans-serif']=['SimHei'] #用来正常显示中文标签
plt.rcParams['axes.unicode_minus']=False #用来正常显示负号

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

# 根据一级目录分别创建文件目录
def initFileDir(filePath):
    fileDirSetting(filePath)  # 要先创建一级目录，之后才能创建二级目录，否则会报错
    fileDirSetting(filePath + 'energy')  # 创建二级目录
    fileDirSetting(filePath + 'strategy')
    fileDirSetting(filePath + 'track')

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
        result = flatten(result)
    return result

# 将list变量存为CSV文件
def writeList2CSV(myList,filePath):
    writer = pd.DataFrame(data=myList) #先把list转化为panda的frame格式，然后存为csv
    writer.to_csv(filePath, encoding='gbk')

# 将列表内容转为词典，列表必须为二维列表
def List2Dict(colorList):
    keyList = colorList[0]
    valueList = colorList[1]
    colorDict = dict(zip(keyList, valueList))
    return colorDict

#输出无人机列表信息
def uavListPrint(uavList):
    for j in range(len(uavList)):
        uav = uavList[j]
        uav.printState()

#输出无人车列表信息
def ugvListPrint(ugvList):
    for j in range(len(ugvList)):
        ugv = ugvList[j]
        ugv.printState()

# 从起始点到目标点画一个带箭头的直线
def plot_vector2d(startPoint, endPoint, **options):
    deltaX = endPoint[0] - startPoint[0]
    deltaY = endPoint[1] - startPoint[1]
    return plt.arrow(startPoint[0], startPoint[1],
                     deltaX, deltaY,
                     head_width=0.2, head_length=0.3,
                     length_includes_head=True, **options)

# 从无人车状态列表中找当前时刻的系统状态
def getStateList(stateList, index):
    tempStateList = stateList[index]
    targetList = np.array(tempStateList)
    return targetList

# 初始化无人机列表
def initUAVList(uavStateList):
    uavList = list()
    for i in range(paraTags.uavNum):
        currentTrackId = -1
        historyTrackIdList = list()
        uavState = uavStateList[i]
        historyEnergyList = list()
        uav = UAV(i, currentTrackId, historyTrackIdList, uavState[0], historyEnergyList, uavState[1], uavState[2], 0)
        uavList.append(uav)
    return uavList

# 初始化无人车列表
def initUGVList():
    ugvList = list()
    for k in range(paraTags.ugvNum):
        ugvId = k
        ugvFormerState = -1
        ugvCurrentState = -1
        follow_UAV_List = list()
        ugv = UGV(ugvId, ugvFormerState, ugvCurrentState, follow_UAV_List)
        ugvList.append(ugv)
    return ugvList

if __name__ == "__main__":
    # 读取事先生成的电量和状态列表信息
    ugvStateList = readCSV2List(paraTags.statePath)
    uavStateList = readCSV2List(paraTags.energyPath)
    distanceList = readCSV2List(paraTags.graphLengthPath)
    colorList = readCSV2List(paraTags.colorPath)
    colorDict = List2Dict(colorList)
    #初始化无人机列表
    uavList = initUAVList(uavStateList)
    #初始化无人车列表
    ugvList = initUGVList()
    #更新无人车和无人机信息
    tempStateList = ugvStateList[0]
    initStateList = np.array(tempStateList)
    for k in range(len(initStateList)):
        ug = ugvList[k]
        ug.formerState = ug.currentState
        ug.currentState = initStateList[k]
        if ug.currentState == 0:
            continue
        for ua in uavList:
            if ua.currentTrackId == -1:
                #代表此时无人机没有跟踪无人车
                ug.follow_UAV_Id_List.append(ua.Id)
                ua.currentTrackId = ug.Id
                ugvFollowSize = len(ug.follow_UAV_Id_List)
                if ugvFollowSize == ug.currentState:
                    break
    # 进入调度算法环节
    if paraTags.algChoose == 1:
        filePath = 'First_Come_First_Out_XYL/'
        initFileDir(filePath)
        broker = Broker(uavList, ugvList, ugvStateList,
                        distanceList, colorDict, filePath)
        broker.First_Come_First_Out_XYL()
    elif paraTags.algChoose == 2:
        filePath = 'Energy_High_First_Out_XYL/'
        initFileDir(filePath)
        broker = Broker(uavList, ugvList, ugvStateList,
                        distanceList, colorDict, filePath)
        broker.Energy_High_First_Out_XYL()
    elif paraTags.algChoose == 3:
        filePath = 'First_Come_First_Out_Greedy/'
        initFileDir(filePath)
        broker = Broker(uavList, ugvList, ugvStateList,
                        distanceList, colorDict, filePath)
        broker.First_Come_First_Out_Greedy()
    elif paraTags.algChoose == 4:
        filePath = 'Energy_High_First_Out_Greedy/'
        initFileDir(filePath)
        broker = Broker(uavList, ugvList, ugvStateList,
                        distanceList, colorDict, filePath)
        broker.Energy_High_First_Out_Greedy()
    elif paraTags.algChoose == 5:
        filePath = 'First_Come_First_Out_Random/'
        initFileDir(filePath)
        broker = Broker(uavList, ugvList, ugvStateList,
                        distanceList, colorDict, filePath)
        broker.First_Come_First_Out_Random()
    elif paraTags.algChoose == 6:
        filePath = 'Energy_High_First_Out_Random/'
        initFileDir(filePath)
        broker = Broker(uavList, ugvList, ugvStateList,
                        distanceList, colorDict, filePath)
        broker.Energy_High_First_Out_Random()
    else:
        print('没有选择任何算法')
