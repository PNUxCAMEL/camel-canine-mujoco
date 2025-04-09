import numpy as np

np.set_printoptions(precision=3)

class simData:
    q = np.zeros(12)     # 12
    qd = np.zeros(12)    # 24
    quat = np.zeros(4)   # 28
    gyro = np.zeros(3)   # 31
    arm_q = np.zeros(6)  # 37
    arm_qd = np.zeros(6) # 43
    grp_q = np.zeros(2)  # 45
    grp_qd = np.zeros(2) # 47

class controlData:
    tau_ref = np.zeros(12)
    tau_arm_ref = np.zeros(6)
    tau_grp_ref = np.zeros(2)

class SharedMemoryManager:
    def __init__(self):
        self.simData = simData()
        self.controlData = controlData()
        self.isTCPConnected = False