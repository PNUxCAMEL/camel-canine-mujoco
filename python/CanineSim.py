import mujoco.viewer
import mujoco
import numpy as np
import time
from collections import deque

D2R = 3.141592 / 180.0

FOOT_BODY_NAMES = ["FL_calf", "FR_calf", "HL_calf", "HR_calf"]
FOOT_TIP_OFFSET = np.array([0.0, 0.0, -0.23])
TRAJ_COLOR_OLD = np.array([1.0, 1.0, 0.0, 0.6], dtype=np.float32)  # 오래된 점: 노란색
TRAJ_COLOR_NEW = np.array([0.0, 1.0, 0.3, 1.0], dtype=np.float32)  # 최신 점: 파란색
TRAJ_DURATION    = 0.3   # 궤적 표시 시간 (초)
TRAJ_INTERVAL    = 0.02  # 점 기록 간격 (초) — 클수록 띄엄띄엄
TRAJ_LEN         = int(TRAJ_DURATION / TRAJ_INTERVAL)
TRAJ_SPHERE_SIZE = np.array([0.008, 0.0, 0.0])
_EYE3            = np.eye(3).flatten()


def _init_foot_traj(model):
    ids  = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in FOOT_BODY_NAMES]
    bufs = [deque(maxlen=TRAJ_LEN) for _ in range(4)]
    return ids, bufs


def _update_foot_traj(data, foot_ids, foot_traj, last_traj_time):
    if data.time - last_traj_time < TRAJ_INTERVAL:
        return last_traj_time
    for i, fid in enumerate(foot_ids):
        pos = data.xpos[fid] + data.xmat[fid].reshape(3, 3) @ FOOT_TIP_OFFSET
        foot_traj[i].append(pos.copy())
    return data.time


def _draw_foot_traj(viewer, foot_traj):
    viewer.user_scn.ngeom = 0
    for traj in foot_traj:
        n = len(traj)
        for j, pos in enumerate(traj):
            if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
                return
            t = j / n  # 0 = 오래된 점(노랑), 1 = 최신 점(파랑)
            color = (1.0 - t) * TRAJ_COLOR_OLD + t * TRAJ_COLOR_NEW
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[viewer.user_scn.ngeom],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                TRAJ_SPHERE_SIZE, pos, _EYE3, color,
            )
            viewer.user_scn.ngeom += 1


def _draw_body_arrow(viewer, data, base_id):
    if viewer.user_scn.ngeom >= viewer.user_scn.maxgeom:
        return
    R   = data.xmat[base_id].reshape(3, 3)
    fwd = R[:, 0]           # body +X (전방) in world
    up  = R[:, 2]           # body +Z (위) in world
    # arrow는 geom 로컬 +Z 방향을 가리키므로, 로컬 Z = fwd 로 재구성
    ax_z = fwd
    ax_x = np.cross(up, fwd)
    ax_y = up
    arrow_mat = np.column_stack([ax_x, ax_y, ax_z]).flatten()
    pos = data.xpos[base_id] + np.array([0.0, 0.0, 0.12])
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[viewer.user_scn.ngeom],
        mujoco.mjtGeom.mjGEOM_ARROW,
        np.array([0.015, 0.015, 0.30]),
        pos, arrow_mat,
        np.array([0.0, 0.5, 1.0, 1.0], dtype=np.float32),
    )
    viewer.user_scn.ngeom += 1


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
        self.foot_ids, self.foot_traj = _init_foot_traj(self.m)
        self.last_traj_time = 0.0
        self.base_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    def startSimulation(self):
        render_dt = 1/60  # 60Hz rendering
        last_render_time = time.time()
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            viewer.cam.type        = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = self.base_id
            viewer.cam.distance    = 2.0
            viewer.cam.azimuth     = -120
            viewer.cam.elevation   = -20
            while viewer.is_running():
                step_start = time.time()

                self.d.ctrl[:] = self.shm.controlData.tau_ref
                mujoco.mj_step(self.m, self.d)
                self.shm.simData.q = self.d.qpos[7:]
                self.shm.simData.qd = self.d.qvel[6:]
                self.shm.simData.quat = self.d.qpos[3:7]
                self.shm.simData.gyro = self.d.qvel[3:6]

                self.last_traj_time = _update_foot_traj(self.d, self.foot_ids, self.foot_traj, self.last_traj_time)

                # rendering control
                if (time.time() - last_render_time) >= render_dt:
                    _draw_foot_traj(viewer, self.foot_traj)
                    _draw_body_arrow(viewer, self.d, self.base_id)
                    viewer.sync()
                    last_render_time = time.time()

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
        self.foot_ids, self.foot_traj = _init_foot_traj(self.m)
        self.last_traj_time = 0.0
        self.base_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    def startSimulation(self):
        render_dt = 1/60  # 60Hz rendering
        last_render_time = time.time()
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            viewer.cam.type        = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = self.base_id
            viewer.cam.distance    = 2.0
            viewer.cam.azimuth     = -120
            viewer.cam.elevation   = -20
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

                self.last_traj_time = _update_foot_traj(self.d, self.foot_ids, self.foot_traj, self.last_traj_time)

                # rendering control
                if (time.time() - last_render_time) >= render_dt:
                    _draw_foot_traj(viewer, self.foot_traj)
                    _draw_body_arrow(viewer, self.d, self.base_id)
                    viewer.sync()
                    last_render_time = time.time()

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
        self.foot_ids, self.foot_traj = _init_foot_traj(self.m)
        self.last_traj_time = 0.0
        self.base_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    def startSimulation(self):
        render_dt = 1/60  # 60Hz rendering
        last_render_time = time.time()
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            viewer.cam.type        = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = self.base_id
            viewer.cam.distance    = 2.0
            viewer.cam.azimuth     = -120
            viewer.cam.elevation   = -20
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

                self.last_traj_time = _update_foot_traj(self.d, self.foot_ids, self.foot_traj, self.last_traj_time)

                # rendering control
                if (time.time() - last_render_time) >= render_dt:
                    _draw_foot_traj(viewer, self.foot_traj)
                    _draw_body_arrow(viewer, self.d, self.base_id)
                    viewer.sync()
                    last_render_time = time.time()

                time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
                else:
                    print("[Mujoco] Deadline Miss, Canine Mujoco Sim. Thread : ",-1.0*time_until_next_step*1e3,"ms")