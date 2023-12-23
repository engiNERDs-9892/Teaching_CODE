package org.firstinspires.ftc.teamcode.drive.Variables;


import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlooppyFloop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Autonomous_Variables {
    public static DcMotor motorLiftyLift;
    public static DcMotor motorRiseyRise;
    public static Servo FlippyFlip;
    public static Servo FlooppyFloop;
    public static Servo GearServo;
    public static Servo LeftClaw;
    public static Servo RightClaw;
    public static Servo AirplaneServo;
    public static Servo HookL;
    public static Servo HookR;

    public static double GroundArmLRotate = 0.03;
    public static double GroundArmRRotate = 0.97 ;
    public static double BackboardArmLRotate = 0.5;
    public static double BackboardArmRRotate = 0.5;

    public static double WristRotateGround = 0.98;
    public static double WristRotateBackboard = 0.55;
    public static double ClawR_Open = 0.77;
    public static double ClawR_Close = 1;
    public static double ClawL_Open = 0.33;
    public static double ClawL_Close = 0;
    }
