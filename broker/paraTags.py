# -*- encoding: utf-8 -*-
'''
@File        :   paraTags.py
@Description :   实验参数设置
@Time        :   2020/08/13 12:12:30
@Author      :   Yan Hui 
@Version     :   1.0
@Contact     :   yanhui13@nudt.edu.cn
'''
class paraTags:
    # 定义控制实验的相关参数
    # **********系统参数**********
    simNum = 100  # 仿真次数
    trackDis = 0.1  # 两辆无人车之间距离按照100米算
    deadline = 10 # 无人机到达指定目标位置的截止期
    distance_upper = 4 # 两辆无人车之间距离上限
    distance_lower = 2 # 两辆无人车之间距离下限
    connectivity = 0.8
    algChoose = 4 # 算法选择
    # **********读写路径**********
    statePath = 'Experiment_Settings/stateList.csv'
    energyPath = 'Experiment_Settings/energyList.csv'
    colorPath = 'Experiment_Settings/colorList.csv'
    graphLengthPath = 'Experiment_Settings/graphLengthList.csv'
    graphTrackPath = 'Experiment_Settings/graphTrackList.csv'
    energyStatisticsPath = 'energy/Statistics.csv'
    trackDisStatisticsPath = 'track/TrackDisStatistics.csv'
    trackTimeStatisticsPath = 'track/TrackTimeStatistics.csv'
    # **********无人车参数**********
    ugvNum = 5  # 无人车数量
    priorityLower = 1  # 无人车优先级下限
    priorityUpper = int(ugvNum * 0.8) + 1  # 无人车优先级上限
    # **********无人机参数**********
    uavNum = 10  # 无人机数量
    energyThreshold = 0.1 # 无人机的安全电量阈值
    # I型无人机的参数
    uavEnergy1 = 45  # 当前时刻的电量，按照初始时刻电量45Wh计
    uavSpeed1 = 25  # 按照25km/h的速度飞
    uavEnergyPower1 = 90  # 刚好是45Wh的电量够飞30分钟
    # II型无人机的参数
    uavEnergy2 = 60  # 当前时刻的电量，按照初始时刻电量45Wh计
    uavSpeed2 = 20  # 按照25km/h的速度飞
    uavEnergyPower2 = 60  # 刚好是90Wh的电量够飞60分钟
    # III型无人机的参数
    uavEnergy3 = 70  # 当前时刻的电量，按照初始时刻电量175Wh计
    uavSpeed3 = 30  # 按照25km/h的速度飞
    uavEnergyPower3 = 105  # 刚好是175Wh的电量够飞40分钟
