# -*- encoding: utf-8 -*-
'''
@File        :   Broker.py
@Description :   调度算法
@Time        :   2020/08/13 12:13:10
@Author      :   Yan Hui 
@Version     :   1.0
@Contact     :   yanhui13@nudt.edu.cn
'''
import numpy as np  # 使用import导入模块numpy，并简写成np
import matplotlib.pyplot as plt  # 使用import导入模块matplotlib.pyplot，并简写成plt
from UAV import UAV
from UGV import UGV
from paraTags import paraTags
import pandas as pd
import graph
import optimizer as opt
plt.rcParams['font.sans-serif']=['SimHei'] #用来正常显示中文标签
plt.rcParams['axes.unicode_minus']=False #用来正常显示负号

class Broker:
    # 定义类的变量
    uavList = list()
    ugvList = list()
    stateList = [[]]
    distanceList = [[]]
    colorDict = {}
    filePath = ''
    # 定义初始化函数
    def __init__(self, uavList, ugvList, stateList, distanceList, colorDict, filePath):
        self.uavList = uavList
        self.ugvList = ugvList
        self.stateList = stateList
        self.distanceList = distanceList
        self.colorDict = colorDict
        self.filePath = filePath
    # ********定义调度策略********
    def First_Come_First_Out_XYL(self):
        """
        @description:
        会议论文先到先出策略
        @param:
        类成员函数
        @Returns:
        无返回
        """
        # 从这里进入正式循环仿真过程
        formerStateList = getStateList(self.stateList, 0)
        simNum = 1
        while simNum <= paraTags.simNum:
            print('*********第 %s 轮仿真*********' % simNum)
            currentStateList = getStateList(self.stateList, simNum)
            # 系统状态变化，相应地更新无人车列表状态
            for c in range(len(currentStateList)):
                ug = self.ugvList[c]
                ug.formerState = ug.currentState
                ug.currentState = currentStateList[c]
            ugvListPrint(self.ugvList)
            stateDelta = stateChange(formerStateList, currentStateList)
            if stateJudge(stateDelta) is False:
                print('系统状态未变化')
                for ua in self.uavList:
                    ua.historyTrackIdList.append(ua.currentTrackId)
                    ua.historyEnergyList.append(ua.currentEnergy)
                # 画调度结果图
                strategyShow(self.uavList, self.colorDict,
                             simNum, self.filePath)
                # 更新系统状态
                simNum += 1  # 无论系统状态是否发生变化都算一轮仿真
                continue
            # 系统状态发生了变化
            print('*********任务调度阶段*********')
            strategyList = generateStrategyList(
                formerStateList, currentStateList, self.distanceList, self.ugvList)
            clusterStrategyList = generateClusterStrategyList(strategyList)
            clusterUavList = []
            for k in range(len(clusterStrategyList)):
                ugv_Id = clusterStrategyList[k][0][0]
                ug = IdToUGV(ugv_Id, self.ugvList)
                clusterUavList.append(ug.follow_UAV_Id_List)  
            clusterStrategyList = clusterListZip(
                clusterUavList, clusterStrategyList)
            print('聚类后的任务调度策略 ', clusterStrategyList)
            # 下面按照聚类后的调度策略进行任务调度
            for k in range(len(clusterStrategyList)):
                clusterTemp = clusterStrategyList[k]
                for j in range(len(clusterTemp)):
                    strategy = clusterTemp[j]
                    if len(strategy) != 4:
                        print('无人机调度策略格式不正确')
                    # 依次读取当前无人车Id，目标无人车Id，执行任务无人机Id
                    currentUgvId = strategy[0]
                    currentUgv = IdToUGV(currentUgvId, self.ugvList)
                    targetUgvId = strategy[1]
                    targetUgv = IdToUGV(targetUgvId, self.ugvList)
                    uaId = strategy[2]
                    moveDistance = strategy[3]
                    ua = IdToUAV(uaId, self.uavList)
                    # 进行无人车操作
                    if currentUgvId != targetUgvId:
                        currentUgv.follow_UAV_Id_List.remove(uaId)  # 离开当前无人机
                        targetUgv.follow_UAV_Id_List.append(uaId)  # 加入目标无人机
                    # 进行无人机操作
                    ua.historyTrackIdList.append(currentUgvId)
                    ua.historyEnergyList.append(ua.currentEnergy)
                    ua.currentTrackId = targetUgvId
                    moveTime = round(moveDistance / ua.speed, 3)  # 计算无人机的飞行时长
                    ua.motionTime += moveTime
                    energyCost = round(moveTime * ua.energyPower, 3)  # 计算无人机的飞行能耗
                    ua.currentEnergy = ua.currentEnergy - energyCost
            print('********无人机和无人车列表信息更新********')
            uavListPrint(self.uavList)
            ugvListPrint(self.ugvList)
            # 画调度结果图
            strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
            # 更新系统状态
            formerStateList = currentStateList
            simNum += 1
            # 当系统中出现某个无人机电量低于阈值则仿真结束
            if uavEenergyJudge(self.uavList) is False:
                break
            # 画每一架无人机跟踪状态图
        uavTrackShow(self.uavList, self.colorDict, simNum, self.filePath)
        overallTrackDisList, overallTrackTimeList, overallEnergyList = uavStatistics(
            self.uavList)
        statisticsTrendShow(overallTrackDisList, overallTrackTimeList,
                            overallEnergyList, self.colorDict, self.filePath)
        trackDisDict, trackTimeDict, energyDict = generateStatisticsDict(
            overallTrackDisList, overallTrackTimeList, overallEnergyList)
        drawStatisticsBox(trackDisDict, trackTimeDict,
                          energyDict, self.filePath)
        print('仿真次数： ', simNum)

    def Energy_High_First_Out_XYL(self):
        """
        @description:
        会议论文先到先出策略
        @param:
        类成员函数
        @Returns:
        无返回
        """
        # 从这里进入正式循环仿真过程
        formerStateList = getStateList(self.stateList, 0)
        simNum = 1
        while simNum <= paraTags.simNum:
            print('*********第 %s 轮仿真*********' % simNum)
            currentStateList = getStateList(self.stateList, simNum)
            # 系统状态变化，相应地更新无人车列表状态
            for c in range(len(currentStateList)):
                ug = self.ugvList[c]
                ug.formerState = ug.currentState
                ug.currentState = currentStateList[c]
            ugvListPrint(self.ugvList)
            stateDelta = stateChange(formerStateList, currentStateList)
            if stateJudge(stateDelta) is False:
                print('系统状态未变化')
                for ua in self.uavList:
                    ua.historyTrackIdList.append(ua.currentTrackId)
                    ua.historyEnergyList.append(ua.currentEnergy)
                # 画调度结果图
                strategyShow(self.uavList, self.colorDict,
                             simNum, self.filePath)
                # 更新系统状态
                simNum += 1  # 无论系统状态是否发生变化都算一轮仿真
                continue
            # 系统状态发生了变化
            print('*********任务调度阶段*********')
            strategyList = generateStrategyList(
                formerStateList, currentStateList, self.distanceList, self.ugvList)
            clusterStrategyList = generateClusterStrategyList(strategyList)
            clusterUavList = []
            for k in range(len(clusterStrategyList)):
                ugv_Id = clusterStrategyList[k][0][0]
                ug = IdToUGV(ugv_Id, self.ugvList)
                follow_id_list = energySort(ug.follow_UAV_Id_List, self.uavList)
                clusterUavList.append(follow_id_list)  
            clusterStrategyList = clusterListZip(
                clusterUavList, clusterStrategyList)
            print('聚类后的任务调度策略 ', clusterStrategyList)
            # 下面按照聚类后的调度策略进行任务调度
            for k in range(len(clusterStrategyList)):
                clusterTemp = clusterStrategyList[k]
                for j in range(len(clusterTemp)):
                    strategy = clusterTemp[j]
                    if len(strategy) != 4:
                        print('无人机调度策略格式不正确')
                    # 依次读取当前无人车Id，目标无人车Id，执行任务无人机Id
                    currentUgvId = strategy[0]
                    currentUgv = IdToUGV(currentUgvId, self.ugvList)
                    targetUgvId = strategy[1]
                    targetUgv = IdToUGV(targetUgvId, self.ugvList)
                    uaId = strategy[2]
                    moveDistance = strategy[3]
                    ua = IdToUAV(uaId, self.uavList)
                    # 进行无人车操作
                    if currentUgvId != targetUgvId:
                        currentUgv.follow_UAV_Id_List.remove(uaId)  # 离开当前无人机
                        targetUgv.follow_UAV_Id_List.append(uaId)  # 加入目标无人机
                    # 进行无人机操作
                    ua.historyTrackIdList.append(currentUgvId)
                    ua.historyEnergyList.append(ua.currentEnergy)
                    ua.currentTrackId = targetUgvId
                    moveTime = round(moveDistance / ua.speed, 3)  # 计算无人机的飞行时长
                    ua.motionTime += moveTime
                    energyCost = round(moveTime * ua.energyPower, 3)  # 计算无人机的飞行能耗
                    ua.currentEnergy = ua.currentEnergy - energyCost
            print('********无人机和无人车列表信息更新********')
            uavListPrint(self.uavList)
            ugvListPrint(self.ugvList)
            # 画调度结果图
            strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
            # 更新系统状态
            formerStateList = currentStateList
            simNum += 1
            # 当系统中出现某个无人机电量低于阈值则仿真结束
            if uavEenergyJudge(self.uavList) is False:
                break
            # 画每一架无人机跟踪状态图
        uavTrackShow(self.uavList, self.colorDict, simNum, self.filePath)
        overallTrackDisList, overallTrackTimeList, overallEnergyList = uavStatistics(
            self.uavList)
        statisticsTrendShow(overallTrackDisList, overallTrackTimeList,
                            overallEnergyList, self.colorDict, self.filePath)
        trackDisDict, trackTimeDict, energyDict = generateStatisticsDict(
            overallTrackDisList, overallTrackTimeList, overallEnergyList)
        drawStatisticsBox(trackDisDict, trackTimeDict,
                          energyDict, self.filePath)
        print('仿真次数： ', simNum)

    def First_Come_First_Out_Greedy(self):
        """
        @description:
        会议论文先到先出策略
        @param:
        类成员函数
        @Returns:
        无返回
        """
        # 从这里进入正式循环仿真过程
        formerStateList = getStateList(self.stateList, 0)
        simNum = 1
        while simNum <= paraTags.simNum:
            print('*********第 %s 轮仿真*********' % simNum)
            currentStateList = getStateList(self.stateList, simNum)
            # 系统状态变化，相应地更新无人车列表状态
            for c in range(len(currentStateList)):
                ug = self.ugvList[c]
                ug.formerState = ug.currentState
                ug.currentState = currentStateList[c]
            ugvListPrint(self.ugvList)
            stateDelta = stateChange(formerStateList, currentStateList)
            if stateJudge(stateDelta) is False:
                print('系统状态未变化')
                for ua in self.uavList:
                    ua.historyTrackIdList.append(ua.currentTrackId)
                    ua.historyEnergyList.append(ua.currentEnergy)
                # 画调度结果图
                strategyShow(self.uavList, self.colorDict,
                             simNum, self.filePath)
                # 更新系统状态
                simNum += 1  # 无论系统状态是否发生变化都算一轮仿真
                continue
            # 系统状态发生了变化
            print('*********任务调度阶段*********')
            strategyList = generateStrategyList(
                formerStateList, currentStateList, self.distanceList, self.ugvList)
            clusterStrategyList = generateClusterStrategyList(strategyList)
            clusterUavList = []
            for k in range(len(clusterStrategyList)):
                ugv_Id = clusterStrategyList[k][0][0]
                ug = IdToUGV(ugv_Id, self.ugvList)
                follow_id_list = energySort(ug.follow_UAV_Id_List, self.uavList)
                clusterUavList.append(follow_id_list)  
            clusterStrategyList = clusterListZip(
                clusterUavList, clusterStrategyList)
            print('聚类后的任务调度策略 ', clusterStrategyList)
            # 下面按照聚类后的调度策略进行任务调度
            for k in range(len(clusterStrategyList)):
                clusterTemp = clusterStrategyList[k]
                for j in range(len(clusterTemp)):
                    strategy = clusterTemp[j]
                    if len(strategy) != 4:
                        print('无人机调度策略格式不正确')
                    # 依次读取当前无人车Id，目标无人车Id，执行任务无人机Id
                    currentUgvId = strategy[0]
                    currentUgv = IdToUGV(currentUgvId, self.ugvList)
                    targetUgvId = strategy[1]
                    targetUgv = IdToUGV(targetUgvId, self.ugvList)
                    uaId = strategy[2]
                    moveDistance = strategy[3]
                    ua = IdToUAV(uaId, self.uavList)
                    # 进行无人车操作
                    if currentUgvId != targetUgvId:
                        currentUgv.follow_UAV_Id_List.remove(uaId)  # 离开当前无人机
                        targetUgv.follow_UAV_Id_List.append(uaId)  # 加入目标无人机
                    # 进行无人机操作
                    ua.historyTrackIdList.append(currentUgvId)
                    ua.historyEnergyList.append(ua.currentEnergy)
                    ua.currentTrackId = targetUgvId
                    moveTime = round(moveDistance / ua.speed, 3)  # 计算无人机的飞行时长
                    ua.motionTime += moveTime
                    energyCost = round(moveTime * ua.energyPower, 3)  # 计算无人机的飞行能耗
                    ua.currentEnergy = ua.currentEnergy - energyCost
            print('********无人机和无人车列表信息更新********')
            uavListPrint(self.uavList)
            ugvListPrint(self.ugvList)
            # 画调度结果图
            strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
            # 更新系统状态
            formerStateList = currentStateList
            simNum += 1
            # 当系统中出现某个无人机电量低于阈值则仿真结束
            if uavEenergyJudge(self.uavList) is False:
                break
            # 画每一架无人机跟踪状态图
        uavTrackShow(self.uavList, self.colorDict, simNum, self.filePath)
        overallTrackDisList, overallTrackTimeList, overallEnergyList = uavStatistics(
            self.uavList)
        statisticsTrendShow(overallTrackDisList, overallTrackTimeList,
                            overallEnergyList, self.colorDict, self.filePath)
        trackDisDict, trackTimeDict, energyDict = generateStatisticsDict(
            overallTrackDisList, overallTrackTimeList, overallEnergyList)
        drawStatisticsBox(trackDisDict, trackTimeDict,
                          energyDict, self.filePath)
        print('仿真次数： ', simNum)

# 将无人车的跟踪无人机列表和无人机聚类调度策略对应的合并，返回合并后的聚类策略
def clusterListZip(uavList,clusterList):
    """
    @description:
    合并无人机列表与对应无人机聚类后调度策略
    @param:
    无人机列表，聚类调度列表
    @Returns:
    合并后的聚类策略
    """
    if len(clusterList) == len(uavList):
        for m in range(len(clusterList)):
            strategy = clusterList[m]
            uavIdList = uavList[m]
            if len(strategy) == len(uavIdList):
                for i in range(len(strategy)):
                    uavStrategy = strategy[i]
                    uavId = uavIdList[i]
                    uavStrategy.insert(2,uavId)
            else:
                print('无人车上跟踪无人机数量与调度策略数量不一致')
    return clusterList

def getStateList(stateList, index):
    """
    @description:
    获得第index轮的目标点重要度状态列表
    @param:
    实验设置生成的状态总表，index
    @Returns:
    np.array类型的状态列表
    """
    tempStateList = stateList[index]
    targetList = np.array(tempStateList)
    return targetList

def stateChange(priorState, currentState):
    """
    @description:
    计算两个状态的差
    @param:
    上一时刻状态和当前状态
    @Returns:
    状态差
    """
    delta = currentState - priorState
    return delta

def stateJudge(stateDelta):
    """
    @description:
    判断系统状态是否发生了变化
    @param:
    状态差列表
    @Returns:
    False：未变化；True：变化
    """
    judgeReuslt= False
    for j in range(len(stateDelta)):
        if stateDelta[j] != 0:
            judgeReuslt = True
            break
    return judgeReuslt

def uavListPrint(uavList):
    """
    @description:
    输出无人机列表信息
    @param:
    无人机列表
    @Returns:
    无返回
    """
    for j in range(len(uavList)):
        uav = uavList[j]
        uav.printState()

def ugvListPrint(ugvList):
    """
    @description:
    输出无人车列表信息
    @param:
    无人车列表
    @Returns:
    无返回
    """
    for j in range(len(ugvList)):
        ugv = ugvList[j]
        ugv.printState()

def uavEenergyJudge(uavList):
    """
    @description:
    判断无人机电量状态
    @param:
    无人机
    @Returns:
    True:无人机电量正常；False：无人机电量低于阈值
    """
    flag = True
    for i in range(len(uavList)):
        ua = uavList[i]
        energyThreshold = ua.historyEnergyList[0] * paraTags.energyThreshold
        if ua.currentEnergy < energyThreshold:
            flag = False
            break
    return flag

def generateStateMatrix(stateList, uavNum):
    """
    @description:
    根据目标重要度状态列表，生成无人机状态向量矩阵
    @param:
    目标重要度状态列表，无人机数量
    @Returns:
    无人机状态向量矩阵
    """
    index = 0
    systemList = []
    for i in range(len(stateList)):
        length = index + stateList[i]
        tempList = []
        for j in range(uavNum):
            if j < index:
                tempList.append(0)
            elif j < length:
                tempList.append(1)
            else:
                tempList.append(0)
        index = length
        systemList.append(tempList)
    systemMatrix = np.array(systemList)
    return systemMatrix

def generateStrategyList(formerStateList, currentStateList, distanceList, ugvList):
    """
    @description:
    生成无人机调度策略列表
    @param:
    目标上一时刻状态列表，目标当前时刻状态列表，无人车列表
    @Returns:
    无人机调度策略：[当前Id, 目标Id, 调度成本]
    """
    distance_matrix = np.array(distanceList)
    # print('原始距离矩阵：\n', distance_matrix)
    delta_state = stateChange(formerStateList, currentStateList)
    cost_matrix, out_id_list, in_id_list = opt.construct_cost_matrix(
        delta_state, distance_matrix)
    # print('out_id_list: ', out_id_list)
    # print('in_id_list: ', in_id_list)
    # print('优化目标矩阵：\n', cost_matrix)
    out_id, in_id, cost_index, _ = opt.xiong_ya_li(cost_matrix)
    # print('输出方id：', out_id)  # 开销矩阵对应的行索引,对应于输出无人机的区域的列表指示器，需要与对应区域的ID映射
    # print('输入方id：', in_id)  # 对应行索引的最优指派的列索引，对应于输入无人机的区域的列表指示器，需要与对应区域的ID映射
    # # 提取每个行索引的最优指派列索引所在的元素的值索引，对应于无人机从输出点到输入点的飞行路径，需要与无人机绑定
    # print('对应cost：', cost_index)
    # print('总体cost：', total_cost)  # 提取每个行索引的最优指派列索引所在的元素值的综合，表示本次规划无人机共需要飞行的距离
    system_strategy = []
    for i in range(len(out_id)):
        former = out_id_list[out_id[i]]
        current = in_id_list[in_id[i]]
        cost = cost_index[i]
        system_strategy.append([former, current, cost])
    print(system_strategy)
    system_strategy = strategy_completion(
        system_strategy, formerStateList, currentStateList)
    return system_strategy

def strategy_completion(system_strategy, formerStateList, currentStateList):
    """
    @description:
    根据状态信息，补全无人机调度策略
    @param:
    基础无人机调度策略，之前状态列表，当前状态列表
    @Returns:
    补全后的无人机调度策略
    """
    delta_state = stateChange(formerStateList, currentStateList)
    for k in range(len(delta_state)):
        if delta_state[k] >= 0:
            m = 0
            while m < formerStateList[k]:
                system_strategy.append([k, k, 0])
                m += 1
        if delta_state[k] < 0:
            m = 0
            while m < currentStateList[k]:
                system_strategy.append([k, k, 0])
                m += 1
    # 按照无人机当前跟踪目标id从小到大对列表排序
    for i in range(len(system_strategy)):
        for j in range(0, len(system_strategy) - i - 1):
            if system_strategy[j][0] > system_strategy[j + 1][0]:
                system_strategy[j], system_strategy[j + 1] = system_strategy[j + 1], system_strategy[j]
    print(system_strategy)
    return system_strategy

def generateClusterStrategyList(strategyList):
    """
    @description:
    按照无人机当前跟踪的目标ID对调度策略进行分类,对于同一类策略按照无人机需要支援距离排序
    @param:
    调度策略
    @Returns:
    按照目标ID分类后的调度策略
    """
    clusterList = []
    index = strategyList[0][0]
    cluster = []
    for k in range(len(strategyList)):
        strategyTemp = strategyList[k]
        if strategyTemp[0] == index:
            cluster.append(strategyTemp)
        else:
            clusterList.append(cluster)
            cluster = []
            cluster.append(strategyTemp)
            index = strategyTemp[0]
        if k == len(strategyList) - 1:
            clusterList.append(cluster)
    # 按照无人机需要支援的距离，对ClusterStrategyList进行排序
    for k in range(len(clusterList)):
        cluster = clusterList[k]
        n = len(cluster)
        for i in range(n):
            for j in range(0, n - i - 1):
                if cluster[j][-1] < cluster[j + 1][-1]:
                    cluster[j], cluster[j + 1] = cluster[j + 1], cluster[j]
    return clusterList

def IdToUAV(ID, UAVlist):
    """
    @description:
    根据无人机ID找到对应无人机类变量
    @param:
    无人机ID,无人机列表
    @Returns:
    对应ID的无人机
    """
    uavResult = None
    for uav in UAVlist:
        if uav.Id ==ID:
            uavResult = uav
            break
    return uavResult

def IdToUGV(ID, UGVlist):
    """
    @description:
    根据无人车ID找到对应无人车类变量
    @param:
    无人车ID,无人车列表
    @Returns:
    对应ID的无人车
    """
    ugvResult = None
    for ugv in UGVlist:
        if ugv.Id ==ID:
            # ugvResult = copy.copy(ugv)
            ugvResult = ugv
            break
    return ugvResult

# 展示算法对于无人机调度结果
def strategyShow(uavList, colorDict, simNum, filePath):
    yBase = 0
    figName = filePath + 'strategy/Round '+ str(simNum) + '.jpg'  # 后缀名改为.pdf就会对应生成pdf文件
    for ua in uavList:
        style = colorDict[str(ua.Id)]  # 根据无人机的Id寻找对应的画图颜色
        xStart = ua.historyTrackIdList[-1]
        xEnd = ua.currentTrackId
        startPoint = np.array([xStart, yBase])
        endPoint = np.array([xEnd, yBase + 0.000001])
        plot_vector2d(startPoint, endPoint, color=style)  # 从起始点到目标点画一个带箭头的直线
        yBase += 1
        plt.xticks(np.arange(-1, paraTags.ugvNum + 1, 1))
        plt.yticks(np.arange(-1, paraTags.uavNum + 1, 1))
        plt.title(filePath[:-1])
        plt.xlabel('无人车编号')
        plt.ylabel('无人机策略')
    plt.grid()
    plt.savefig(figName)  # sava需要在show之前，不然会保存图片报错
    plt.close()

# 展示无人机的历史运动轨迹
def uavTrackShow(uavList, colorDict, simNum, filePath):
    overallTrackIdList = []
    for ua in uavList:
        trackIdList = ua.historyTrackIdList
        trackIdList.append(ua.currentTrackId)
        overallTrackIdList.append(trackIdList)
        yBase = 1
        Id = ua.Id
        style = colorDict[str(ua.Id)]  # 根据无人机的Id寻找对应的画图颜色
        # print('%s 号无人机的历史跟踪轨迹是 %s ' %(ua.Id, ua.historyTrackIdList))
        figName = filePath + 'track/UAV ' + str(Id) + '.jpg' # 后缀名改为.pdf就会对应生成pdf文件
        for k in range(len(ua.historyTrackIdList)):
            if k < len(ua.historyTrackIdList) - 1:
                xStart = ua.historyTrackIdList[k]
                xEnd = ua.historyTrackIdList[k + 1]
                startPoint = np.array([xStart, yBase])
                endPoint = np.array([xEnd, yBase + 0.000001])
                plot_vector2d(startPoint, endPoint, color=style)
                yBase += 1
            else:
                xStart = ua.historyTrackIdList[k]
                xEnd = ua.currentTrackId
                startPoint = np.array([xStart, yBase])
                endPoint = np.array([xEnd, yBase + 0.000001])
                plot_vector2d(startPoint, endPoint, color=style)
        plt.xticks(np.arange(-1, paraTags.ugvNum + 1, 1))
        plt.yticks(np.arange(0, simNum + 2, 1))
        plt.title(filePath[:-1])
        plt.xlabel('无人车编号')
        plt.ylabel('仿真次数')
        plt.grid()
        plt.savefig(figName)
        plt.close()
    fileName = filePath + 'track/trackIdList.csv'
    writeList2CSV(overallTrackIdList, fileName)

# 展示无人机的历史运动轨迹
def uavStatistics(uavList):
    overallTrackDisList = []
    overallTrackTimeList = []
    overallEnergyList = []
    for ua in uavList:
        trackDisList = []
        trackTimeList = []
        accumulativeTrackDis = 0
        accumulativeTrackTime = 0
        uavHistoryEnergyList = ua.historyEnergyList
        uavHistoryEnergyList.append(ua.currentEnergy)
        overallEnergyList.append(uavHistoryEnergyList)
        for k in range(len(ua.historyTrackIdList)):
            if k < len(ua.historyTrackIdList) - 1:
                xStart = ua.historyTrackIdList[k]
                xEnd = ua.historyTrackIdList[k + 1]
            else:
                xStart = ua.historyTrackIdList[k]
                xEnd = ua.currentTrackId
            trackDis = abs(xStart - xEnd) * paraTags.trackDis
            trackTime = trackDis / ua.speed
            accumulativeTrackDis += trackDis * 1000
            accumulativeTrackTime += trackTime * 3600
            trackDisList.append(accumulativeTrackDis)
            trackTimeList.append(accumulativeTrackTime)
        overallTrackDisList.append(trackDisList)
        overallTrackTimeList.append(trackTimeList)
    return overallTrackDisList, overallTrackTimeList, overallEnergyList

def statisticsTrendShow(overallTrackDisList,overallTrackTimeList,overallEnergyList,colorDict, filePath):
    fileName = filePath + 'track/trackDisList.csv'
    writeList2CSV(overallTrackDisList, fileName)
    fileName = filePath + 'track/trackTimeList.csv'
    writeList2CSV(overallTrackTimeList, fileName)
    fileName = filePath + 'energy/energyList.csv'
    writeList2CSV(overallEnergyList, fileName)

    figName = filePath + 'track/UAV_TrackDis_Trend.jpg'
    trendShow(overallTrackDisList, colorDict, figName)
    figName = filePath + 'track/UAV_TrackTime_Trend.jpg'
    trendShow(overallTrackTimeList, colorDict, figName)
    figName = filePath + 'energy/UAV_Energy_Trend.jpg'
    trendShow(overallEnergyList, colorDict, figName)

def trendShow(overallTrackList, colorDict, figName):
    maxTrackDis, minTrackDis = find_2DList_Max_Min_Value(overallTrackList)
    for k in range(len(overallTrackList)):
        Id = k
        style = colorDict[str(Id)]  # 根据无人机的Id寻找对应的画图颜色
        trackDisList = overallTrackList[k]
        listLen = len(trackDisList)
        x_axis_data = list(range(1,listLen+1))
        y_axis_data = trackDisList
        pltLabel = 'uav '+ str(Id)
        plt.plot(x_axis_data, y_axis_data, 'ro-', color=style, alpha=1, linewidth=1, label=pltLabel)
        plt.xticks(np.arange(1, listLen+1, 1))
        plt.yticks(np.arange(minTrackDis * 0.8, maxTrackDis * 1.2, round((maxTrackDis - minTrackDis)/10,1)))
        plt.title(figName.split('/')[0])
        plt.xlabel('仿真次数')
        plt.ylabel(figName.split('/')[-1].split('_')[1])
    plt.grid()
    plt.savefig(figName)
    # plt.show()
    plt.close()

# 按照仿真序列生成系统历史电量的词典
def overallList2Dict(overallStatisticsList):
    dataDict = {}
    roundNum = len(overallStatisticsList[0])
    for k in range(roundNum):
        roundList = []
        for i in range(len(overallStatisticsList)):
            statisticsList = overallStatisticsList[i]
            roundList.append(statisticsList[k])
        id = str(k+1)
        dataDict[id] = roundList
    return dataDict

def generateStatisticsDict(overallTrackDisList, overallTrackTimeList,overallEnergyList):
    trackDisDict = overallList2Dict(overallTrackDisList)
    trackTimeDict = overallList2Dict(overallTrackTimeList)
    energyDict = overallList2Dict(overallEnergyList)
    return trackDisDict, trackTimeDict,energyDict

# 系统整体角度每个阶段的电量统计图
def drawBoxPlot(dataDict, figName, statisticsPath):
    print(dataDict)
    print(len(dataDict))
    statisticsName = figName.split('/')[0] + '/' + statisticsPath
    df = pd.DataFrame(dataDict)
    print(df)
    df.plot.box(title=figName.split('/')[0])
    plt.xlabel('仿真次数')
    plt.ylabel(figName.split('/')[-1].split('_')[1])
    plt.grid(linestyle="--", alpha=0.8)
    plt.savefig(figName)
    # 将箱线图的统计参数存为csv格式的文件
    df.describe().to_csv(statisticsName, encoding='gbk')
    print(df.describe())
    plt.show()

def drawStatisticsBox(trackDisDict, trackTimeDict, energyDict, filePath):
    figName = filePath + 'track/UAV_TrackDis_Trend.jpg'
    drawBoxPlot(trackDisDict, figName, paraTags.trackDisStatisticsPath)
    figName = filePath + 'track/UAV_TrackTime_Trend.jpg'
    drawBoxPlot(trackTimeDict, figName, paraTags.trackTimeStatisticsPath)
    figName = filePath + 'energy/System_Energy_Statistics.jpg'
    drawBoxPlot(energyDict, figName, paraTags.energyStatisticsPath)


# 从起始点到目标点画一个带箭头的直线
def plot_vector2d(startPoint, endPoint, **options):
    deltaX = endPoint[0] - startPoint[0]
    deltaY = endPoint[1] - startPoint[1]
    return plt.arrow(startPoint[0], startPoint[1],
                     deltaX, deltaY,
                     head_width=0.2, head_length=0.3,
                     length_includes_head=True, **options)

# 按照电量下降的顺序对无人机列表排序
def energySort(uavIdList, uavList):
    """
    @description:
    按照电量下降的顺序对无人机列表排序
    @param:
    无人机ID列表，无人机列表
    @Returns:
    排序后的无人机ID列表
    """
    n = len(uavIdList)
    # 遍历所有数组元素
    for i in range(n):
        for j in range(0, n - i - 1):
            ua0 = IdToUAV(uavIdList[j], uavList)
            ua1 = IdToUAV(uavIdList[j+1], uavList)
            if ua0.currentEnergy < ua1.currentEnergy:
                uavIdList[j], uavIdList[j+1] = uavIdList[j+1], uavIdList[j]
    return uavIdList

# 找到一个二维列表中最大值和最小值
def find_2DList_Max_Min_Value(data_2DList):
    """
    @description:
    找到一个二维列表中最大值和最小值
    @param:
    二维列表
    @Returns:
    最大值，最小值
    """
    max_data = []
    min_data = []
    for i in range(len(data_2DList)):
        max_data.append(max(data_2DList[i]))
        min_data.append(min(data_2DList[i]))
    return max(max_data), min(min_data)

# 根据统计的每次的失败数计算每次的成功率
def guaranteeRatioRecording(failNumList, filePath):
    guaranteeRatioList = []
    fileName = filePath + 'guaranteeRatio.csv'
    for i in range(len(failNumList)):
        failNum  = failNumList[i]
        guaranteeRatio = (paraTags.uavNum - failNum)/paraTags.uavNum
        guaranteeRatioList.append(guaranteeRatio)
    writeList2CSV(guaranteeRatioList, fileName)

# 将list变量存为CSV文件
def writeList2CSV(myList,filePath):
    """
    @description:
    将列表中数据存为csv数据格式文件
    @param:
    列表，存储文件路径
    @Returns:
    无返回
    """
    writer = pd.DataFrame(data=myList) #先把list转化为panda的frame格式，然后存为csv
    writer.to_csv(filePath, encoding='gbk')
