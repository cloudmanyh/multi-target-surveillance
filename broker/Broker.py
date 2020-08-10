import numpy as np  # 使用import导入模块numpy，并简写成np
import matplotlib.pyplot as plt  # 使用import导入模块matplotlib.pyplot，并简写成plt
from UAV import UAV
from UGV import UGV
from paraTags import paraTags
import pandas as pd
plt.rcParams['font.sans-serif']=['SimHei'] #用来正常显示中文标签
plt.rcParams['axes.unicode_minus']=False #用来正常显示负号

class Broker:
    # 定义类的变量
    uavList = list()
    ugvList = list()
    stateList = [[]]
    colorDict = {}
    filePath = ''
    # 定义初始化函数
    def __init__(self, uavList, ugvList, stateList, colorDict, filePath):
        self.uavList = uavList
        self.ugvList = ugvList
        self.stateList = stateList
        self.colorDict = colorDict
        self.filePath = filePath
    # ********定义调度策略********

    # 调度策略：先到先出
    def First_Come_First_Out(self):
        # 从这里进入正式循环仿真过程
        formerStateList = getStateList(self.stateList, 0)
        simNum = 1
        failNumList = []
        while simNum <= paraTags.simNum:
            print('*********第 %s 轮仿真*********' % simNum)
            currentStateList = getStateList(self.stateList, simNum)
            failNum = 0
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
                strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
                # 更新系统状态
                simNum += 1  # 无论系统状态是否发生变化都算一轮仿真
                failNumList.append(failNum)
                continue
            # 系统状态发生了变化
            print('*********无人机解除绑定阶段*********')
            for j in range(len(stateDelta)):
                if stateDelta[j] < 0:  # 处于离开状态
                    print('目标：%s 号无人车撤出 %s 架无人机' % (j, abs(stateDelta[j])))
                    # 无人机撤出采用先到先出模式
                    for ug in self.ugvList:
                        if ug.Id == j:
                            outTrackSize = 0
                            if len(ug.follow_UAV_Id_List) >= abs(stateDelta[j]):
                                # 无人车需要解除跟踪的无人机数目小于目前正在跟踪的数目
                                outTrackSize = abs(stateDelta[j])
                            else:
                                # 无人车需要解除跟踪的无人机数目大于目前正在跟踪的数目
                                print("sth wrong")
                                outTrackSize = len(ug.follow_UAV_Id_List)
                            for out in range(outTrackSize):
                                # 调度思想是按照队头先退出的思想
                                offUAVId = ug.follow_UAV_Id_List[0]
                                del ug.follow_UAV_Id_List[0]
                                ua = IdToUAV(offUAVId, self.uavList)
                                formerTrackId = ua.currentTrackId
                                ua.historyTrackIdList.append(formerTrackId)
                                ua.historyEnergyList.append(ua.currentEnergy)
                                ua.currentTrackId = -1
                                print('策略：%s 号无人机离开 %s 号无人车' % (ua.Id, ug.Id))
            for ua in self.uavList:
                if ua.currentTrackId != -1:  # 对于当前跟踪目标Id没发生变化的情况，意味着无人机保持不变
                    ua.historyTrackIdList.append(ua.currentTrackId)
                    ua.historyEnergyList.append(ua.currentEnergy)
            print('*********无人机解除绑定结果********')
            uavListPrint(self.uavList)
            print('*********无人机重新绑定阶段********')
            for j in range(len(stateDelta)):
                if stateDelta[j] > 0:  # 处于加入状态
                    print('目标：%s 号无人车增加 %s 架无人机' % (j, stateDelta[j]))
                    inTrackSize = stateDelta[j]
                    ug = IdToUGV(j, self.ugvList)
                    for ua in self.uavList:
                        if ua.currentTrackId == -1:
                            ug.follow_UAV_Id_List.append(ua.Id)
                            ua.currentTrackId = ug.Id
                            print('策略：%s 号无人机加入 %s 号无人车' % (ua.Id, ua.currentTrackId))
                            moveDistance = abs(ua.historyTrackIdList[-1] - ua.currentTrackId)* paraTags.trackDis
                            moveTime = round(moveDistance / ua.speed,3) # 保留三维小数
                            ua.motionTime += moveTime
                            if moveTime * 3600 > paraTags.deadline:
                                # 说明无人机飞到无人车边上时已经超时
                                failNum += 1
                            energyCost = round(moveTime * ua.energyPower,3) # 保留三维小数
                            ua.currentEnergy = ua.currentEnergy - energyCost
                            inTrackSize -= 1
                        if inTrackSize == 0:
                            break
            print('********无人机和无人车列表信息更新********')
            uavListPrint(self.uavList)
            ugvListPrint(self.ugvList)
            # 画调度结果图
            strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
            # 更新系统状态
            formerStateList = currentStateList
            failNumList.append(failNum)
            simNum += 1
            # 当系统中出现某个无人机电量低于阈值则仿真结束
            if uavEenergyJudge(self.uavList) is False:
                break
        # 画每一架无人机跟踪状态图
        uavTrackShow(self.uavList, self.colorDict, simNum, self.filePath)
        overallTrackDisList, overallTrackTimeList, overallEnergyList = uavStatistics(self.uavList)
        statisticsTrendShow(overallTrackDisList, overallTrackTimeList, overallEnergyList, self.colorDict, self.filePath)
        trackDisDict, trackTimeDict, energyDict = generateStatisticsDict(overallTrackDisList, overallTrackTimeList,
                                                                         overallEnergyList)
        drawStatisticsBox(trackDisDict, trackTimeDict, energyDict, self.filePath)
        guaranteeRatioRecording(failNumList, self.filePath)
        print('仿真次数： ', simNum)

    def Energy_High_First_Out(self):
        # 从这里进入正式循环仿真过程
        formerStateList = getStateList(self.stateList, 0)
        simNum = 1
        failNumList = []
        while simNum <= paraTags.simNum:
            print('*********第 %s 轮仿真*********' % simNum)
            currentStateList = getStateList(self.stateList, simNum)
            failNum = 0
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
                strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
                # 更细系统状态
                failNumList.append(failNum)
                simNum += 1  # 无论系统状态是否发生变化都算一轮仿真
                continue
            # 系统状态发生了变化
            print('*********无人机解除绑定阶段*********')
            for j in range(len(stateDelta)):
                if stateDelta[j] < 0:  # 处于离开状态
                    print('目标：%s 号无人车撤出 %s 架无人机' % (j, abs(stateDelta[j])))
                    # 无人机撤出采用先到先出模式
                    for ug in self.ugvList:
                        if ug.Id == j:
                            outTrackSize = 0
                            if len(ug.follow_UAV_Id_List) >= abs(stateDelta[j]):
                                # 无人车需要解除跟踪的无人机数目小于目前正在跟踪的数目
                                outTrackSize = abs(stateDelta[j])
                            else:
                                # 无人车需要解除跟踪的无人机数目大于目前正在跟踪的数目
                                print("sth wrong")
                                outTrackSize = len(ug.follow_UAV_Id_List)
                            ug.follow_UAV_Id_List = energySort(ug.follow_UAV_Id_List, self.uavList)
                            for out in range(outTrackSize):
                                # 调度思想是按照电量高的先退出的思想
                                offUAVId = ug.follow_UAV_Id_List[0]
                                del ug.follow_UAV_Id_List[0]
                                ua = IdToUAV(offUAVId, self.uavList)
                                formerTrackId = ua.currentTrackId
                                ua.historyTrackIdList.append(formerTrackId)
                                ua.historyEnergyList.append(ua.currentEnergy)
                                ua.currentTrackId = -1
                                print('策略：%s 号无人机离开 %s 号无人车' % (ua.Id, ug.Id))
            for ua in self.uavList:
                if ua.currentTrackId != -1:  # 对于当前跟踪目标Id没发生变化的情况，意味着无人机保持不变
                    ua.historyTrackIdList.append(ua.currentTrackId)
                    ua.historyEnergyList.append(ua.currentEnergy)
            print('*********无人机解除绑定结果********')
            uavListPrint(self.uavList)
            print('*********无人机重新绑定阶段********')
            for j in range(len(stateDelta)):
                if stateDelta[j] > 0:  # 处于加入状态
                    print('目标：%s 号无人车增加 %s 架无人机' % (j, stateDelta[j]))
                    inTrackSize = stateDelta[j]
                    ug = IdToUGV(j, self.ugvList)
                    for ua in self.uavList:
                        if ua.currentTrackId == -1:
                            ug.follow_UAV_Id_List.append(ua.Id)
                            ua.currentTrackId = ug.Id
                            print('策略：%s 号无人机加入 %s 号无人车' % (ua.Id, ua.currentTrackId))
                            moveDistance = abs(ua.historyTrackIdList[-1] - ua.currentTrackId) * paraTags.trackDis
                            moveTime = round(moveDistance / ua.speed,3) # 保留三维小数
                            ua.motionTime += moveTime
                            if moveTime * 3600 > paraTags.deadline:
                                # 说明无人机飞到无人车边上时已经超时
                                failNum += 1
                            energyCost = round(moveTime * ua.energyPower,3) # 保留三维小数
                            ua.currentEnergy = ua.currentEnergy - energyCost
                            inTrackSize -= 1
                            if inTrackSize == 0:
                                break
            print('********无人机和无人车列表信息更新********')
            uavListPrint(self.uavList)
            ugvListPrint(self.ugvList)
            # 画调度结果图
            strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
            # 更新系统状态
            formerStateList = currentStateList
            failNumList.append(failNum)
            simNum += 1
            # 当系统中出现某个无人机电量低于阈值则仿真结束
            if uavEenergyJudge(self.uavList) is False:
                break
        # 画每一架无人机跟踪状态图
        uavTrackShow(self.uavList, self.colorDict, simNum, self.filePath)
        overallTrackDisList, overallTrackTimeList, overallEnergyList = uavStatistics(self.uavList)
        statisticsTrendShow(overallTrackDisList, overallTrackTimeList, overallEnergyList, self.colorDict, self.filePath)
        trackDisDict, trackTimeDict, energyDict = generateStatisticsDict(overallTrackDisList, overallTrackTimeList,
                                                                         overallEnergyList)
        drawStatisticsBox(trackDisDict, trackTimeDict, energyDict, self.filePath)
        guaranteeRatioRecording(failNumList, self.filePath)
        print('仿真次数： ', simNum)

    def Load_Balance_By_Cooperation(self):
        # 从这里进入正式循环仿真过程
        formerStateList = getStateList(self.stateList, 0)
        simNum = 1
        failNumList = []
        while simNum <= paraTags.simNum:
            print('*********第 %s 轮仿真*********' % simNum)
            currentStateList = getStateList(self.stateList, simNum)
            failNum = 0
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
                strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
                # 更新系统状态
                failNumList.append(failNum)
                simNum += 1  # 无论系统状态是否发生变化都算一轮仿真
                continue
            # 系统状态发生了变化
            strategyList = generateStrategyList(formerStateList, currentStateList, paraTags.uavNum)
            for k in range(len(strategyList)):
                ua = IdToUAV(k, self.uavList)
                uavStrategy = strategyList[k]
                currentId = uavStrategy[0]
                targetId = uavStrategy[1]
                if targetId == currentId:
                    # 说明此时无人机不进行行动调整
                    ua.historyTrackIdList.append(ua.currentTrackId)
                    ua.historyEnergyList.append(ua.currentEnergy)
                else:
                    # 进行无人车操作
                    if ua.currentTrackId == currentId:
                        currentUgv = IdToUGV(currentId, self.ugvList)
                        targetUgv = IdToUGV(targetId, self.ugvList)
                        currentUgv.follow_UAV_Id_List.remove(ua.Id)  # 离开当前无人机
                        targetUgv.follow_UAV_Id_List.append(ua.Id)  # 加入目标无人机
                        # 进行无人机操作
                        formerTrackId = ua.currentTrackId
                        ua.historyTrackIdList.append(formerTrackId)
                        ua.historyEnergyList.append(ua.currentEnergy)
                        ua.currentTrackId = targetId
                        moveDistance = abs(currentId - targetId)* paraTags.trackDis
                        moveTime = round(moveDistance / ua.speed,3) # 保留三维小数
                        ua.motionTime += moveTime
                        if moveTime * 3600 > paraTags.deadline:
                            # 说明无人机飞到无人车边上时已经超时
                            failNum += 1
                        energyCost = round(moveTime * ua.energyPower,3) # 保留三维小数
                        ua.currentEnergy = ua.currentEnergy - energyCost
                    else:
                        print('状态信息更新出现混乱')
            print('********无人机和无人车列表信息更新********')
            uavListPrint(self.uavList)
            ugvListPrint(self.ugvList)
            # 画调度结果图
            strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
            # 更新系统状态
            formerStateList = currentStateList
            failNumList.append(failNum)
            simNum += 1
            # 当系统中出现某个无人机电量低于阈值则仿真结束
            if uavEenergyJudge(self.uavList) is False:
                break
        # 画每一架无人机跟踪状态图
        uavTrackShow(self.uavList, self.colorDict, simNum, self.filePath)
        overallTrackDisList, overallTrackTimeList, overallEnergyList = uavStatistics(self.uavList)
        statisticsTrendShow(overallTrackDisList, overallTrackTimeList, overallEnergyList, self.colorDict, self.filePath)
        trackDisDict, trackTimeDict, energyDict = generateStatisticsDict(overallTrackDisList, overallTrackTimeList,
                                                                         overallEnergyList)
        drawStatisticsBox(trackDisDict, trackTimeDict, energyDict, self.filePath)
        guaranteeRatioRecording(failNumList, self.filePath)
        print('仿真次数： ', simNum)

    def Load_Balance_Energy_Efficient(self):
        # 从这里进入正式循环仿真过程
        formerStateList = getStateList(self.stateList, 0)
        simNum = 1
        failNumList = []
        while simNum <= paraTags.simNum:
            print('*********第 %s 轮仿真*********' % simNum)
            currentStateList = getStateList(self.stateList, simNum)
            failNum = 0
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
                strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
                # 更新系统状态
                failNumList.append(failNum)
                simNum += 1  # 无论系统状态是否发生变化都算一轮仿真
                continue
            # 系统状态发生了变化
            print('*********任务调度阶段*********')
            strategyList = generateStrategyList(formerStateList, currentStateList, paraTags.uavNum)
            clusterStrategyList = generateClusterStrategyList(strategyList)
            clusterUavList = []
            for k in range(len(self.ugvList)):
                ug = self.ugvList[k]
                if len(ug.follow_UAV_Id_List) > 0:
                    tempList = energySort(ug.follow_UAV_Id_List, self.uavList)
                    clusterUavList.append(tempList)
            clusterStrategyList = clusterListZip(clusterUavList, clusterStrategyList)
            print('聚类后的任务调度策略 ',clusterStrategyList)
            # 下面按照聚类后的调度策略进行任务调度
            for k in range(len(clusterStrategyList)):
                clusterTemp = clusterStrategyList[k]
                for j in range(len(clusterTemp)):
                    strategy = clusterTemp[j]
                    if len(strategy) != 3:
                        print('无人机调度策略格式不正确')
                    # 依次读取当前无人车Id，目标无人车Id，执行任务无人机Id
                    currentUgvId = strategy[0]
                    currentUgv = IdToUGV(currentUgvId, self.ugvList)
                    targetUgvId = strategy[1]
                    targetUgv = IdToUGV(targetUgvId, self.ugvList)
                    uaId = strategy[2]
                    ua = IdToUAV(uaId, self.uavList)
                    # 进行无人车操作
                    currentUgv.follow_UAV_Id_List.remove(uaId)  # 离开当前无人机
                    targetUgv.follow_UAV_Id_List.append(uaId)  # 加入目标无人机
                    # 进行无人机操作
                    ua.historyTrackIdList.append(currentUgvId)
                    ua.historyEnergyList.append(ua.currentEnergy)
                    ua.currentTrackId = targetUgvId
                    moveDistance = abs(currentUgvId - targetUgvId)* paraTags.trackDis
                    moveTime = round(moveDistance / ua.speed,3) # 保留三维小数
                    ua.motionTime += moveTime
                    if moveTime * 3600 > paraTags.deadline:
                        # 说明无人机飞到无人车边上时已经超时
                        failNum += 1
                    energyCost = round(moveTime * ua.energyPower,3) # 保留三维小数
                    ua.currentEnergy = ua.currentEnergy - energyCost
            print('********无人机和无人车列表信息更新********')
            uavListPrint(self.uavList)
            ugvListPrint(self.ugvList)
            # 画调度结果图
            strategyShow(self.uavList, self.colorDict, simNum, self.filePath)
            # 更新系统状态
            formerStateList = currentStateList
            failNumList.append(failNum)
            simNum += 1
            # 当系统中出现某个无人机电量低于阈值则仿真结束
            if uavEenergyJudge(self.uavList) is False:
                break
            # 画每一架无人机跟踪状态图
        uavTrackShow(self.uavList, self.colorDict, simNum, self.filePath)
        overallTrackDisList, overallTrackTimeList, overallEnergyList = uavStatistics(self.uavList)
        statisticsTrendShow(overallTrackDisList, overallTrackTimeList,overallEnergyList, self.colorDict, self.filePath)
        trackDisDict, trackTimeDict, energyDict = generateStatisticsDict(overallTrackDisList, overallTrackTimeList, overallEnergyList)
        drawStatisticsBox(trackDisDict, trackTimeDict, energyDict, self.filePath)
        guaranteeRatioRecording(failNumList, self.filePath)
        print('仿真次数： ',simNum)

# 将无人车的跟踪无人机列表和无人机聚类调度策略对应的合并，返回合并后的聚类策略
def clusterListZip(uavList,clusterList):
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
    else:
        print('无人车聚类与调度策略聚类数量不一致')
    return clusterList


# 从无人车状态列表中找当前时刻的系统状态
def getStateList(stateList, index):
    tempStateList = stateList[index]
    targetList = np.array(tempStateList)
    return targetList

#计算两个状态差，状态差代表着简单调度策略
def stateChange(priorState,currentState):
    delta = currentState - priorState
    return delta

#判断系统状态是否发生变化
def stateJudge(stateDelta):
    judgeReuslt= False
    for j in range(len(stateDelta)):
        if stateDelta[j] != 0:
            judgeReuslt = True
            break
    return judgeReuslt

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

# 判断无人机列表中是否出现某个无人机电量低于阈值
def uavEenergyJudge(uavList):
    flag = True
    for i in range(len(uavList)):
        ua = uavList[i]
        energyThreshold = ua.historyEnergyList[0] * paraTags.energyThreshold
        if ua.currentEnergy < energyThreshold:
            flag = False
    return flag

# 根据无人车每时每刻的系统状态，生成无人机状态向量矩阵
def generateStateMatrix(stateList, listLen):
    index = 0
    systemList = []
    for i in range(len(stateList)):
        length = index + stateList[i]
        tempList = []
        for j in range(listLen):
            if j < index:
                tempList.append(0)
            elif j < length:
                tempList.append(1)
            else:
                tempList.append(0)
        index = length
        systemList.append(tempList)
    formerSystemMatrix = np.array(systemList)
    return formerSystemMatrix

# 产生系统调度策略列表，对应每一个无人机的调度策略数据格式：[当前Id，目标Id]
def generateStrategyList(formerStateList, currentStateList, uavNum):
    formerStateMatrix = generateStateMatrix(formerStateList, uavNum)
    currentStateMatrix = generateStateMatrix(currentStateList, uavNum)
    deltaStateMatrix = currentStateMatrix - formerStateMatrix
    strategyList = []
    for j in range(uavNum):
        uavStateChange = deltaStateMatrix[:, j]
        formerState = formerStateMatrix[:,j]
        uavStrategy = []
        if np.all(uavStateChange == 0):
            formerIndex = formerState.tolist().index(1) # 从无人机上一时刻状态表中寻找对应无人车Id
            currentId, targetId =formerIndex, formerIndex  # 表示无人机的位置信息不发生变化
        else:
            uavStateChangeList = uavStateChange.tolist()
            currentId = uavStateChangeList.index(-1)
            targetId = uavStateChangeList.index(1)
        uavStrategy.append(currentId)
        uavStrategy.append(targetId)
        strategyList.append(uavStrategy)
    return strategyList

# 按照无人机当前跟踪的无人车ID对调度策略进行分类
def generateClusterStrategyList(strategyList):
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
                strategy0 = cluster[j]
                strategy1 = cluster[j + 1]
                if abs(strategy0[0] - strategy0[1]) < abs(strategy1[0] - strategy1[1]):
                    cluster[j], cluster[j + 1] = cluster[j + 1], cluster[j]
    return clusterList

#通过无人机ID找到对应的无人机类变量，目前还不够完善
def IdToUAV(ID, UAVlist):
    uavResult = None
    for uav in UAVlist:
        if uav.Id ==ID:
            uavResult = uav
            break
    return uavResult

#通过无人车ID找到对应的无人车类变量，目前还不够完善
def IdToUGV(ID, UGVlist):
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
def energySort(uavIdList,uavList):
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
    '''
    功能：找到矩阵最大值和最小值
    '''
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
    writer = pd.DataFrame(data=myList) #先把list转化为panda的frame格式，然后存为csv
    writer.to_csv(filePath, encoding='gbk')
