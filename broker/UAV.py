# 类定义
class UAV:
    # 定义基本属性
    Id = -1  # 无人机的Id
    currentTrackId = -1 # 当前时刻跟踪的无人车Id
    historyTrackIdList = list()
    currentEnergy = 45 # 当前时刻的电量，按照初始时刻电量45Wh计
    historyEnergyList = list()
    speed = 0 # 无人机的机动速度
    energyPower = 0 # 无人机的能耗功率
    motionTime = 0 # 无人机的运动时长
    # 定义构造方法
    def __init__(self, Id, currentTrackId, historyTrackIdList, initEnergy, historyEnergyList, energyPower, speed, motionTime):
        self.Id= Id
        self.currentTrackId = currentTrackId
        self.historyTrackIdList = historyTrackIdList
        self.currentEnergy = initEnergy
        self.historyEnergyList = historyEnergyList
        self.energyPower = energyPower
        self.speed = speed
        self.motionTime = motionTime
    def printState(self):
        print("%s 号无人机之前跟踪 %s 号无人车目前跟踪 %s 号无人车,之前电量 %s 当前电量 %s 累计飞行机动时长 %s " %
              (self.Id, self.historyTrackIdList[-1], self.currentTrackId, round(self.historyEnergyList[-1],3), round(self.currentEnergy,3), round(self.motionTime,3)))

if __name__ == "__main__":
    # 实例化类
    UAVId = 1
