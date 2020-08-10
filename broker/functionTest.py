import numpy as np  # 使用import导入模块numpy，并简写成np
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
stateList = [1,3,2,4]
systemMatrix = generateStateMatrix(stateList, 10)
print(systemMatrix)