#include "console/JoystickCommand.hpp"

JoystickCommand::JoystickCommand()
{
    joystickCommand = GAMEPAD_NO_INPUT;
    gamepad = Gamepad::getInstance();
    sharedMemory = SharedMemory::getInstance();
    sharedMemory->gamepad.controllerName = std::string(gamepad->name_of_joystick);
}

void JoystickCommand::JoystickCommandFunction()
{
    while (true)
    {
        gamepad->Read();
        sendJoystickinfo();
        mappingFunction();

        sharedMemory->gamepad.joyCommand = joystickCommand;
        std::copy(bodyAngVel_ref, bodyAngVel_ref + 3, sharedMemory->gamepad.userAngVel.data());
        std::copy(bodyLinVel_ref, bodyLinVel_ref + 3, sharedMemory->gamepad.userLinVel.data());
    }
}

void JoystickCommand::sendJoystickinfo()
{
    sharedMemory->gamepad.joystick.LeftStickX = (double)gamepad->mJoystickAxis[0] / 30767;
    sharedMemory->gamepad.joystick.LeftStickY = -(double)gamepad->mJoystickAxis[1] / 30767;
    sharedMemory->gamepad.joystick.LeftTrigger = gamepad->mJoystickAxis[2] / 30767;
    sharedMemory->gamepad.joystick.RightStickX = (double)gamepad->mJoystickAxis[3] / 30767;
    sharedMemory->gamepad.joystick.RightStickY = -(double)gamepad->mJoystickAxis[4] / 30767;
    sharedMemory->gamepad.joystick.RightTrigger = gamepad->mJoystickAxis[5] / 30767;
    sharedMemory->gamepad.joystick.DpadX = gamepad->mJoystickAxis[6] / 30767;
    sharedMemory->gamepad.joystick.DpadY = -gamepad->mJoystickAxis[7] / 30767;
    sharedMemory->gamepad.button.A = gamepad->mJoystickButton[0];
    sharedMemory->gamepad.button.B = gamepad->mJoystickButton[1];
    sharedMemory->gamepad.button.X = gamepad->mJoystickButton[2];
    sharedMemory->gamepad.button.Y = gamepad->mJoystickButton[3];
    sharedMemory->gamepad.button.LB = gamepad->mJoystickButton[4];
    sharedMemory->gamepad.button.RB = gamepad->mJoystickButton[5];
    sharedMemory->gamepad.button.Back = gamepad->mJoystickButton[6];
    sharedMemory->gamepad.button.Start = gamepad->mJoystickButton[7];
    sharedMemory->gamepad.button.Guide = gamepad->mJoystickButton[8];
    sharedMemory->gamepad.button.LeftStick = gamepad->mJoystickButton[9];
    sharedMemory->gamepad.button.RightStick = gamepad->mJoystickButton[10];
}

void JoystickCommand::mappingFunction()
{
    joystickCommand = GAMEPAD_NO_INPUT;
//    mappingStart();
//    mappingEmergencyStop();
//    mappingStandUp();
//    mappingStandDown();
//    mappingMotorOff();
//    mappingTrotStop();
//    mappingTrotSlow();
//    mappingTrotFast();
//    mappingTrotOverlap();
//    mappingRecovery();
    mappingJoystick();
    mappingRestart();
    switch (sharedMemory->FSMState)
    {
    case FSM_INITIAL:
        mappingStart();
        mappingEmergencyStop();
        break;
    case FSM_READY:
        mappingStandUp();
        mappingEmergencyStop();
        break;
    case FSM_STAND_UP:
        mappingEmergencyStop();
        break;
    case FSM_SIT_DOWN:
        mappingEmergencyStop();
        break;
    case FSM_STAND:
        mappingStandDown();
        mappingTrotSlow();
        mappingTrotFast();
        mappingTrotOverlap();
        mappingEmergencyStop();
        break;
    case FSM_TROT_STOP:
        mappingTrotStop();
        mappingEmergencyStop();
        break;
    case FSM_TROT_SLOW:
        mappingTrotStop();
        mappingTrotFast();
        mappingTrotOverlap();
        mappingEmergencyStop();
        break;
    case FSM_TROT_FAST:
        mappingTrotStop();
        mappingTrotSlow();
        mappingTrotFast();
        mappingTrotOverlap();
        mappingEmergencyStop();
        break;
    case FSM_OVERLAP_TROT_FAST:
        mappingTrotStop();
        mappingTrotSlow();
        mappingTrotFast();
        mappingEmergencyStop();
        break;
    case FSM_EMERGENCY_STOP:
        mappingRestart();
        break;
    case FSM_RESTART:
        break;
    default:
        break;
    }

    switch(sharedMemory->armFSMState)
    {
    case ARM_FSM::ARM_INITIAL:
        break;
    case ARM_FSM::ARM_MOTOR_READY:
    {
        mappingArmHome();
        break;
    }
    case ARM_FSM::ARM_E_STOP:
    {
        mappingArmHome();
        break;
    }
    case ARM_FSM::ARM_HOME:
    {
        mappingArmTeleOn();
        mappingArmTeleOff();
        mappingArmMove();
        mappingArmGripperOpen();
        mappingArmGripperClose();
        break;
    }
    case ARM_FSM::ARM_TELE:
    {
        mappingArmTeleOn();
        mappingArmTeleOff();
        mappingArmGripperOpen();
        mappingArmGripperClose();
        if(!sharedMemory->isArmTele)
        {
            mappingArmHome();
            mappingArmMove();
        }
        break;
    }
    case ARM_FSM::ARM_MOVE:
    {

        break;
    }
    case ARM_FSM::ARM_READY:
    {
        mappingArmHome();
        mappingArmMove();
        mappingArmTeleOn();
        mappingArmTeleOff();
        mappingArmGripperOpen();
        mappingArmGripperClose();
        break;
    }
    default:
        break;
    }
}

void JoystickCommand::mappingStart()
{
    if (sharedMemory->gamepad.button.B || sharedMemory->gamepad.gui.GUIButton == GUI_START)
    {
        joystickCommand = GAMEPAD_START;
    }
}

void JoystickCommand::mappingEmergencyStop()
{
    if ((sharedMemory->gamepad.joystick.LeftTrigger > 0.5 && sharedMemory->gamepad.joystick.RightTrigger > 0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_E_STOP)
    {
        joystickCommand = GAMEPAD_EMERGENCY_STOP;
    }
}

void JoystickCommand::mappingStandUp()
{
    if ((sharedMemory->gamepad.joystick.RightTrigger > 0.5 && sharedMemory->gamepad.joystick.DpadY > 0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_HOME_UP)
    {
        joystickCommand = GAMEPAD_STAND_UP;
    }
}

void JoystickCommand::mappingStandDown()
{
    if ((sharedMemory->gamepad.joystick.RightTrigger > 0.5 && sharedMemory->gamepad.joystick.DpadY < -0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_HOME_DOWN)
    {
        joystickCommand = GAMEPAD_SIT_DOWN;
    }
}

void JoystickCommand::mappingTrotStop()
{
    if ((sharedMemory->gamepad.button.RB && sharedMemory->gamepad.joystick.DpadX > 0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_TROT_STOP)
    {
        joystickCommand = GAMEPAD_TROT_STOP;
    }
}

void JoystickCommand::mappingTrotSlow()
{
    if ((sharedMemory->gamepad.button.RB && sharedMemory->gamepad.joystick.DpadX < -0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_TROT_SLOW)
    {
        joystickCommand = GAMEPAD_TROT_SLOW;
    }
}

void JoystickCommand::mappingTrotFast()
{
    if ((sharedMemory->gamepad.button.RB && sharedMemory->gamepad.joystick.DpadY < -0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_TROT_FAST)
    {
        joystickCommand = GAMEPAD_TROT_FAST;
    }
}

void JoystickCommand::mappingTrotOverlap()
{
    if ((sharedMemory->gamepad.button.RB && sharedMemory->gamepad.joystick.DpadY > 0.5) || sharedMemory->gamepad.gui.GUIButton == GUI_TROT_OVERLAP)
    {
        joystickCommand = GAMEPAD_TROT_OVERLAP;
    }
}

void JoystickCommand::mappingRestart()
{
    if (sharedMemory->gamepad.gui.GUIButton == GUI_RESTART || sharedMemory->gamepad.button.B)
    {
        joystickCommand = GAMEPAD_RESTART;
    }
}

void JoystickCommand::mappingArmHome()
{
    if (sharedMemory->gamepad.gui.GUIButton == GUI_ARM_HOME)
    {
        joystickCommand = GAMEPAD_ARM_HOME;
    }
}

void JoystickCommand::mappingArmMove()
{
    if (sharedMemory->gamepad.gui.GUIButton == GUI_ARM_MOVE)
    {
        joystickCommand = GAMEPAD_ARM_MOVE;
    }
}

void JoystickCommand::mappingArmTeleOn()
{
    if (sharedMemory->gamepad.gui.GUIButton == GUI_ARM_TELE_ON)
    {
        joystickCommand = GAMEPAD_ARM_TELE_ON;
    }
}

void JoystickCommand::mappingArmTeleOff()
{
    if (sharedMemory->gamepad.gui.GUIButton == GUI_ARM_TELE_OFF)
    {
        joystickCommand = GAMEPAD_ARM_TELE_OFF;
    }
}

void JoystickCommand::mappingArmGripperOpen()
{
    if (sharedMemory->gamepad.gui.GUIButton == GUI_ARM_GRP_OPEN)
    {
        joystickCommand = GAMEPAD_ARM_GRP_OPEN;
    }
}

void JoystickCommand::mappingArmGripperClose()
{
    if (sharedMemory->gamepad.gui.GUIButton == GUI_ARM_GRP_CLOSE)
    {
        joystickCommand = GAMEPAD_ARM_GRP_CLOSE;
    }
}

void JoystickCommand::mappingJoystick()
{
    switch (sharedMemory->FSMState)
    {
    case FSM_STAND:
    {
//        bodyLinVel_ref[0] = sharedMemory->gamepad.joystick.LeftStickY * 0.6;
//        bodyLinVel_ref[1] = -sharedMemory->gamepad.joystick.LeftStickX * 0.4;
//        bodyAngVel_ref[2] = -sharedMemory->gamepad.joystick.RightStickX * 0.65;
        break;
    }
    case FSM_TROT_SLOW:
    {
        bodyLinVel_ref[0] = sharedMemory->gamepad.joystick.LeftStickY * 0.6;
        bodyLinVel_ref[1] = -sharedMemory->gamepad.joystick.LeftStickX * 0.4;
        bodyAngVel_ref[2] = -sharedMemory->gamepad.joystick.RightStickX * 0.65;
        break;
    }
    case FSM_TROT_FAST:
    {
        bodyLinVel_ref[0] = sharedMemory->gamepad.joystick.LeftStickY * 1.0; // 1 m/s
        bodyLinVel_ref[1] = -sharedMemory->gamepad.joystick.LeftStickX * 0.5;
        bodyAngVel_ref[2] = -sharedMemory->gamepad.joystick.RightStickX * 0.65;
        break;
    }
    case FSM_OVERLAP_TROT_FAST:
    {
        bodyLinVel_ref[0] = sharedMemory->gamepad.joystick.LeftStickY * 1.0;
        bodyAngVel_ref[2] = -sharedMemory->gamepad.joystick.RightStickX * 0.65;
        break;
    }
    default:
        break;

    }

    double armLinearVelocityLimit[3] = {0.0};
    double armAngularVelocityLimit[3] = {0.0};
    double armLinearVelocityRef[3] = {0.0};
    double armAngularVelocityRef[3] = {0.0};

    switch(sharedMemory->armFSMState)
    {
        case ARM_FSM::ARM_TELE:
        {
            armAngularVelocityLimit[0] = 0.5;
            armAngularVelocityLimit[1] = 0.5;
            armAngularVelocityLimit[2] = 0.5;
            armLinearVelocityLimit[0] = 0.5;
            armLinearVelocityLimit[1] = 0.5;
            armLinearVelocityLimit[2] = 0.5;

            if(sharedMemory->gamepad.joystick.LeftTrigger > 0.5)
            {
                armLinearVelocityRef[0] = 0.0; // x
                armLinearVelocityRef[1] = 0.0; // y
                armLinearVelocityRef[2] = sharedMemory->gamepad.joystick.LeftStickY * 0.2;     // z
            }
            else
            {
                armLinearVelocityRef[0] = sharedMemory->gamepad.joystick.LeftStickY * 0.2;     // x
                armLinearVelocityRef[1] = -sharedMemory->gamepad.joystick.LeftStickX * 0.2;     // y
                armLinearVelocityRef[2] = 0.0; // z
            }

            armAngularVelocityRef[0] = (double)sharedMemory->gamepad.joystick.DpadX * 0.5;         // roll
            armAngularVelocityRef[1] = sharedMemory->gamepad.joystick.RightStickY * 0.5;   // pitch
            armAngularVelocityRef[2] = sharedMemory->gamepad.joystick.RightStickX * 0.5;   // yaw
            break;
        }
        default:
            break;
    }

    sharedMemory->desiredTeleOperationLinearVelocity[0] = fmin(fmax(armLinearVelocityRef[0], -armLinearVelocityLimit[0]), armLinearVelocityLimit[0]);
    sharedMemory->desiredTeleOperationLinearVelocity[1] = fmin(fmax(armLinearVelocityRef[1], -armLinearVelocityLimit[1]), armLinearVelocityLimit[1]);
    sharedMemory->desiredTeleOperationLinearVelocity[2] = fmin(fmax(armLinearVelocityRef[2], -armLinearVelocityLimit[2]), armLinearVelocityLimit[2]);
    sharedMemory->desiredTeleOperationAngularVelocity[0] = fmin(fmax(armAngularVelocityRef[0], -armAngularVelocityLimit[0]), armAngularVelocityLimit[0]);
    sharedMemory->desiredTeleOperationAngularVelocity[1] = fmin(fmax(armAngularVelocityRef[1], -armAngularVelocityLimit[1]), armAngularVelocityLimit[1]);
    sharedMemory->desiredTeleOperationAngularVelocity[2] = fmin(fmax(armAngularVelocityRef[2], -armAngularVelocityLimit[2]), armAngularVelocityLimit[2]);

}