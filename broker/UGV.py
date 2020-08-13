# -*- encoding: utf-8 -*-
'''
@File        :   UGV.py
@Description :   无人车类
@Time        :   2020/08/13 12:17:32
@Author      :   Yan Hui 
@Version     :   1.0
@Contact     :   yanhui13@nudt.edu.cn
'''
class UGV:
    # 定义基本属性
    Id = -1
    formerState = -1
    currentState = -1
    follow_UAV_List = list() # 默认跟随的无人机ID列表
    # 定义构造方法
    def __init__(self, Id, formerState, currentState, follow_UAV_Id_List):
        """
        @description:
        无人车类构造函数
        @param:
        无人车ID，上一时刻状态，当前时刻状态，跟随无人机ID列表
        @Returns:
        无返回
        """
        self.Id = Id
        self.formerState = formerState
        self.currentState = currentState
        self.follow_UAV_Id_List = follow_UAV_Id_List

    def printState(self):
        """
        @description:
        输出无人车基本信息
        @param:
        无输入
        @Returns:
        无返回
        """
        print("%s 号无人车之前状态 %s 当前状态 %s 跟随的无人机列表是%s" % (self.Id, self.formerState, self.currentState, self.follow_UAV_Id_List))


if __name__ == "__main__":
    # 实例化类
    UGVId = 1
    UGVState = 2
    UAV_List = [1,2]
    p = UGV(UGVId, UGVState, UGVState, UAV_List)
    p.printState()

