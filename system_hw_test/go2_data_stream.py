import time
import sys

from unitree.unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree.unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree.unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
import unitree_legged_const as go2

class Custom:
    def __init__(self):
        self.low_state = None  

    # Public methods
    def Init(self):
        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
        print("IMU state: ", msg.imu_state)
        print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)

if __name__ == '__main__':

    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()

    while True:        
        time.sleep(1)