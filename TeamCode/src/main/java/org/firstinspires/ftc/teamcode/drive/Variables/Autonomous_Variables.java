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

    public static double DegreeClaw = 0.00333333333;  // 1/300 (Torque Servo)
    public static double DegreeArm = 0.00333333333; //  1/300 (Torque Servo)
    public static double DegreeAirplane = 0.00333333333; //  1/300 (Torque Servo)
    public static double DegreeWrist = 0.000555555556; // 1/1800 (5 Turn Servo)

    public static double GroundArmLRotate = 9;
    public static double GroundArmRRotate = 291 ;
    public static double BackboardArmLRotate = 90;
    public static double BackboardArmRRotate = 210;

    public static double Stack5ArmLRotate = 45;
    public static double Stack5ArmRRotate = 255; //300-45 Since servo = reversed

    public static double Stack4ArmLRotate = 35;
    public static double Stack4ArmRRotate = 245; //300-35 Since servo = reversed

    public static double Stack3ArmLRotate = 25;
    public static double Stack3ArmRRotate = 275; //300-25 Since servo = reversed

    public static double Stack2ArmLRotate = 15;
    public static double Stack2ArmRRotate = 285; //300-15 Since servo = reversed
    public static double Stack1ArmLRotate = 5;
    public static double Stack1ArmRRotate = 295; //300-5 Since servo = reversed

    public static double WristRotateGround = 0;
    public static double WristRotateStack = 0;
    public static double WristRotateBackboard = 165;

    public static double ClawR_Open = 255;
    public static double ClawR_Close = 300;
    public static double ClawL_Open = 45;
    public static double ClawL_Close = 0;
    }
