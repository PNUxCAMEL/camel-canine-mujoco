import mujoco.viewer
import mujoco
import numpy as np
import time

D2R = 3.141592 / 180.0

class Canine:
    def __init__(self, SharedMemoryManager):
        self.shm = SharedMemoryManager
        self.sim_dt = 0.002 # 500Hz

        # Load robot model
        self.m = mujoco.MjModel.from_xml_path("canine_mjcf/scene_canine.xml")
        self.d = mujoco.MjData(self.m)
        self.m.opt.timestep = self.sim_dt
        # Set initial joint positions
        self.initial_angles = np.array([
            0.0, 126, -160,
            0.0, 126, -160,
            0.0, 126, -160,
            0.0, 126, -160,
        ], dtype=np.float32)
        
    
        self.initial_angles = self.initial_angles * D2R
        self.initial_xyz = np.array([0.0, 0.0, 0.12])
        self.d.qpos[0:3] = self.initial_xyz
        self.d.qpos[7:7+12] = self.initial_angles
        mujoco.mj_step(self.m, self.d)

    def startSimulation(self):
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            while viewer.is_running():
                step_start = time.time()

                self.d.ctrl[:] = self.shm.controlData.tau_ref
                mujoco.mj_step(self.m, self.d)
                self.shm.simData.q = self.d.qpos[7:]
                self.shm.simData.qd = self.d.qvel[6:]
                self.shm.simData.quat = self.d.qpos[3:7]
                self.shm.simData.gyro = self.d.qvel[3:6]
                
                viewer.sync()

                time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                else:
                    print("[Mujoco] Deadline Miss, Canine Mujoco Sim. Thread : ",-1.0*time_until_next_step*1e3,"ms")


class CanineGadget:
    def __init__(self, SharedMemoryManager):
        self.shm = SharedMemoryManager
        self.sim_dt = 0.005 # 200Hz

        # Load robot model
        self.m = mujoco.MjModel.from_xml_path("canine_mjcf/scene_canine_gadget.xml")
        self.d = mujoco.MjData(self.m)

        self.m.opt.timestep = self.sim_dt

        # Set initial joint positions
        self.initial_angles = np.array([
            0.0,  126, -160,
            0.0,  126, -160,
            0.0,  126, -160,
            0.0,  126, -160,
            0.0, -90.0, 120.0, 
            0.0,  0.0, 0.0,
            0.025/D2R,  -0.025/D2R
        ], dtype=np.float32)
        
    
        self.initial_angles = self.initial_angles * D2R
        self.initial_xyz = np.array([0.0, 0.0, 0.08])
        self.d.qpos[0:3] = self.initial_xyz
        self.d.qpos[7:7+12] = self.initial_angles[:12]
        self.d.qpos[7:7+20] = self.initial_angles
        
        mujoco.mj_step(self.m, self.d)

    def startSimulation(self):
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            while viewer.is_running():
                step_start = time.time()

                self.d.ctrl[0:12] = self.shm.controlData.tau_ref
                self.d.ctrl[12:12+6] = self.shm.controlData.tau_arm_ref
                self.d.ctrl[18:18+2] = self.shm.controlData.tau_grp_ref

                mujoco.mj_step(self.m, self.d)
                self.shm.simData.quat = self.d.qpos[3:7]
                self.shm.simData.q = self.d.qpos[7:7+12]
                self.shm.simData.arm_q = self.d.qpos[19:19+6]
                self.shm.simData.grp_q = self.d.qpos[25:25+2]
                
                self.shm.simData.gyro = self.d.qvel[3:6]
                self.shm.simData.qd = self.d.qvel[6:6+12]
                self.shm.simData.arm_qd = self.d.qvel[18:18+6]
                self.shm.simData.grp_qd = self.d.qvel[24:24+2]
                
                viewer.sync()

                time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                else:
                    print("[Mujoco] Deadline Miss, Canine Mujoco Sim. Thread : ",-1.0*time_until_next_step*1e3,"ms")

class CanineGadgetLight:
    def __init__(self, SharedMemoryManager):
        self.shm = SharedMemoryManager
        self.sim_dt = 0.005 # 200Hz

        # Load robot model
        self.m = mujoco.MjModel.from_xml_path("canine_mjcf/scene_canine_gadget_light.xml")
        self.d = mujoco.MjData(self.m)

        self.m.opt.timestep = self.sim_dt

        # Set initial joint positions
        self.initial_angles = np.array([
            0.0,  126, -160,
            0.0,  126, -160,
            0.0,  126, -160,
            0.0,  126, -160,
            0.0, -90.0, 120.0, 
            0.0,  0.0, 0.0,
            0.025/D2R,  -0.025/D2R
        ], dtype=np.float32)
        
    
        self.initial_angles = self.initial_angles * D2R
        self.initial_xyz = np.array([0.0, 0.0, 0.08])
        self.d.qpos[0:3] = self.initial_xyz
        self.d.qpos[7:7+12] = self.initial_angles[:12]
        self.d.qpos[7:7+20] = self.initial_angles
        
        mujoco.mj_step(self.m, self.d)

    def startSimulation(self):
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            while viewer.is_running():
                step_start = time.time()

                self.d.ctrl[0:12] = self.shm.controlData.tau_ref
                self.d.ctrl[12:12+6] = self.shm.controlData.tau_arm_ref
                self.d.ctrl[18:18+2] = self.shm.controlData.tau_grp_ref

                mujoco.mj_step(self.m, self.d)
                self.shm.simData.quat = self.d.qpos[3:7]
                self.shm.simData.q = self.d.qpos[7:7+12]
                self.shm.simData.arm_q = self.d.qpos[19:19+6]
                self.shm.simData.grp_q = self.d.qpos[25:25+2]
                
                self.shm.simData.gyro = self.d.qvel[3:6]
                self.shm.simData.qd = self.d.qvel[6:6+12]
                self.shm.simData.arm_qd = self.d.qvel[18:18+6]
                self.shm.simData.grp_qd = self.d.qvel[24:24+2]
                
                viewer.sync()

                time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                else:
                    print("[Mujoco] Deadline Miss, Canine Mujoco Sim. Thread : ",-1.0*time_until_next_step*1e3,"ms")