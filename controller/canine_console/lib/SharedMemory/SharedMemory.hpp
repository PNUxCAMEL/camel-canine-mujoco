//
// Created by ys on 24. 2. 16.
//

#ifndef RBCANINE_SHAREDMEMORY_HPP
#define RBCANINE_SHAREDMEMORY_HPP

#include <Eigen/Dense>
#include "custom_struct.hpp"
#include "EnumClasses.hpp"
#include "ConsoleType.hpp"
#define MPC_HORIZON         5
#define R2D                 57.2957802

typedef class _HWD_
{
public:
    _HWD_(const _HWD_&) = delete;
    _HWD_& operator=(const _HWD_&) = delete;
    static _HWD_* getInstance();
private:
    _HWD_();
public:
    SENSOR_INFO sensor;
    double motorDesiredTorque[MOTOR_NUM_LEG];
    double motorDesiredPos[MOTOR_NUM_LEG];
    double armMotorDesiredTorque[MOTOR_NUM_ARM];
    double armMotorDesiredPos[MOTOR_NUM_ARM];
    double threadElapsedTime[11];
    bool motorCheckFlag;
    bool motorALLStatus;
} HWD, * pHWD;

typedef class SharedMemory
{
public:
    SharedMemory(const SharedMemory&) = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;

    static SharedMemory* getInstance();
private:
    SharedMemory();
public:
    // For UI
    GAMEPAD gamepad;
    YAMLPARAM yamlParams;


    std::string robotAddress;
    bool isIPChanged;
    bool consoleConnection;

    UI_COMMAND command;
    int gaitTable[MPC_HORIZON*4];

    bool isNan;

    bool SPI2CANStatus;

    bool motorStatus;

    int SPI2CANState;

    int FSMState;
    int LowControlState;

    double localTime;

    int motorErrorStatus[MOTOR_NUM_LEG];
    int motorTemp[MOTOR_NUM_LEG];
    double motorVoltage[MOTOR_NUM_LEG];
    double motorPosition[MOTOR_NUM_LEG];
    double motorVelocity[MOTOR_NUM_LEG];
    double motorTorque[MOTOR_NUM_LEG];
    double motorDesiredPosition[MOTOR_NUM_LEG];
    double motorDesiredVelocity[MOTOR_NUM_LEG];
    double motorDesiredTorque[MOTOR_NUM_LEG];

    Eigen::Vector3d globalBasePosition;
    Eigen::Vector3d globalBaseVelocity;
    Eigen::Vector3d bodyBaseVelocity;
    Eigen::Vector4d globalBaseQuaternion;
    Eigen::Vector3d globalBaseDesiredPosition;
    Eigen::Vector3d globalBaseDesiredVelocity;
    Eigen::Vector4d globalBaseDesiredQuaternion;
    Eigen::Vector3d globalBaseDesiredEulerAngle;
    Eigen::Vector3d bodyBaseDesiredAngularVelocity;
    Eigen::Vector3d globalBaseDesiredAngularVelocity;
    Eigen::Vector3d bodyBaseDesiredVelocity;

    Eigen::Vector3d globalBaseEulerAngle;
    Eigen::Vector3d bodyBaseAngularVelocity;
    Eigen::Vector3d globalBaseAngularVelocity;
    Eigen::Vector3d globalBaseAcceleration;

    Eigen::Vector3d pdTorque[4];
    Eigen::Vector3d mpcTorque[4];
    Eigen::Vector3d globalFootPosition[4];
    Eigen::Vector3d bodyBase2FootPosition[4];
    Eigen::Vector3d bodyBase2FootVelocity[4];
    Eigen::Vector3d bodyBase2FootDesiredPosition[4];
    Eigen::Vector3d bodyBase2FootDesiredVelocity[4];

    double contactResidualTorque[4];
    bool contactState[4];
    bool CartesianContactState[4][3];
    bool earlyLanding[4];
    bool lateLanding[4];
    Eigen::Vector3d solvedGRF[4];

    double testBasePos[3];
    double testBaseVel[3];
    double threadElapsedTime[13];
    // 0: Command, 1: High controller, 2: Low controller,
    // 3: CAN_FL, 4: CAN_FR, 5: CAN_HL, 6: CAN_HR, 7: IMU, 8: Visual
    // 9: PC_client, 10: PC_server
    // 11: Console1, 12: Console2

    // For Controller
    int gaitState;
    double gaitPeriod;
    double swingPeriod;
    double standPeriod;
    bool gaitChangeFlag;
    bool isFirstHome[4];
    bool bSwingLiftDown[4];
    bool bGaitState[4];
    bool bStateDiverged;
    bool bIsEndHome;

    Eigen::Vector3d estimatedGroundSlope;
    double tempRes[4];
    double simulContactForceFL;
    double simulContactForceFR;
    double simulContactForceHL;
    double simulContactForceHR;

    Eigen::Vector3d swingPgain[4];
    Eigen::Vector3d swingDgain[4];

    Eigen::Vector3d mpcDesiredPos[MPC_HORIZON];

    STAIR_KEY_FRAME stairKeyFrame;
    Eigen::Vector3d globalReferencePosition;
    Eigen::Vector3d swingDesiredAcc[4];
    Eigen::Vector3d qddot[6];

    bool paramChangedFlag;
    bool mpcParamChangedFlag;
    bool swingParamChangedFlag;
    bool wbcParamChangedFlag;

    bool swingUpPhase[4];

    Eigen::Vector3d BasePos;
    Eigen::Vector3d BasePos_ref;
    Eigen::Vector3d BaseVel;
    Eigen::Vector3d BaseVel_ref;

    bool BasePosFLAG;

    bool isRobotRestart;

    bool isTCPConnected;
    double TCP_Duration;

    bool stumbleRecovery;

    // for arm control;
    bool GadgetRMDStatus;
    int GadgetRMDState;
    bool GadgetDXStatus;
    int GadgetDXState;
    bool isArmTele;
    int armFSMState;
    int LowArmControlState;
    int armGRPControlMode;
    int armMotorErrorStatus[MOTOR_NUM_ARM];
    int armMotorTemp[MOTOR_NUM_ARM];
    double armMotorVoltage[MOTOR_NUM_ARM];
    double armMotorPosition[MOTOR_NUM_ARM];
    double armMotorVelocity[MOTOR_NUM_ARM];
    double armMotorTorque[MOTOR_NUM_ARM];
    double armMotorDesiredPosition[MOTOR_NUM_ARM];
    double armMotorDesiredVelocity[MOTOR_NUM_ARM];
    double armMotorDesiredTorque[MOTOR_NUM_ARM];
    Eigen::Vector3d currentEndEffectorPosition;
    Eigen::Vector3d currentEndEffectorEulerAngle;
    Eigen::Vector3d currentEndEffectorVelocity;
    Eigen::Vector3d currentEndEffectorAngularVelocity;
    double desiredEndEffectorPosition[3];
    double desiredEndEffectorEulerAngle[3];
    double desiredTeleOperationLinearVelocity[3];
    double desiredTeleOperationAngularVelocity[3];
} SHM, * pSHM;


#endif //RBCANINE_SHAREDMEMORY_HPP
