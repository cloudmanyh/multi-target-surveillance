class UGV:
    # 定义基本属性
    Id = -1
    formerState = -1
    currentState = -1
    follow_UAV_List = list() # 默认跟随的无人机ID列表
    # 定义构造方法
    def __init__(self, Id, formerState, currentState, follow_UAV_Id_List):
        self.Id = Id
        self.formerState = formerState
        self.currentState = currentState
        self.follow_UAV_Id_List = follow_UAV_Id_List

    def printState(self):
        print("%s 号无人车之前状态 %s 当前状态 %s 跟随的无人机列表是%s" % (self.Id, self.formerState, self.currentState, self.follow_UAV_Id_List))


if __name__ == "__main__":
    # 实例化类
    UGVId = 1
    UGVState = 2
    UAV_List = [1,2]
    p = UGV(UGVId, UGVState, UAV_List)
    p.printState()

