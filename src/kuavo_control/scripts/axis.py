#!/usr/bin/env python3

from base import Base

class Axis(Base):
    """坐标变换类
    """
    
    def __init__(self) -> None:
        """坐标变换类类初始化
        """
        super(Axis, self).__init__()
    
    def camera_to_gripper(self):
        """相机到夹爪
        """
        pass
    
    def gripper_to_eef(self):
        """夹爪到末端执行器
        """
        pass
    
    def camera_to_eef(self):
        """相机到末端执行器
        
        camera -> gripper
        gripper -> eef
        => camera -> eef
        """
        pass
        
    
        