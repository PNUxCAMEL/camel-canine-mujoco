import socket
import struct
import time

class CanineTCPClient:
    def __init__(self, SharedMemoryManager):
        self.shm = SharedMemoryManager
        self.connectTCP()
        

    def connectTCP(self):
        while not self.shm.isTCPConnected:
            try:
                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client.connect(("127.0.0.1", 60005))
            except socket.error as e:
                print("[TCP] waiting to connect TCP server.")
                self.shm.isTCPConnected = False
                time.sleep(1)
            else:
                self.shm.isTCPConnected = True
                print("[TCP] Connected to TCP server!")

    def doCommunication(self):
        try:
            tau_refs = self.client.recv(20 * 8)  # 18 doubles (each 8 bytes)
            refs = struct.unpack("20d", tau_refs)
        except struct.error as e:
            self.shm.isTCPConnected = False
            self.connectTCP()
            time.sleep(1)
        except socket.error as e:
            self.shm.isTCPConnected = False
            self.connectTCP()
            time.sleep(1)
        else:
            for i in range(12):
                self.shm.controlData.tau_ref[i] = refs[i]
            for i in range(6):
                self.shm.controlData.tau_arm_ref[i] = refs[12+i]
            for i in range(2):
                self.shm.controlData.tau_grp_ref[i] = refs[18+i]
            states = (
                        [self.shm.simData.q[i] for i in range(12)] +
                        [self.shm.simData.qd[i] for i in range(12)] +
                        [self.shm.simData.quat[i] for i in range(4)] +
                        [self.shm.simData.gyro[i] for i in range(3)] +
                        [self.shm.simData.arm_q[i] for i in range(6)] +
                        [self.shm.simData.arm_qd[i] for i in range(6)] +
                        [self.shm.simData.grp_q[i] for i in range(2)] +
                        [self.shm.simData.grp_qd[i] for i in range(2)]
                    )
            self.client.sendall(struct.pack("47d", *states))




    
        