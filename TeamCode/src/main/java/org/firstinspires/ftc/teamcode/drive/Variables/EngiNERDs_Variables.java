package org.firstinspires.ftc.teamcode.drive.Variables;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class EngiNERDs_Variables {

    public static DcMotor motorFL = null;
    public static DcMotor motorFR = null;
    public static DcMotor motorBL = null;
    public static DcMotor motorBR = null;

    public static DcMotor motorLiftyLift;

    public static DcMotor motorRiseyRise;

    public static Servo FlippyFlip;

    public static Servo FlooppyFloop;

    public static Servo LeftClaw;

    public static Servo RightClaw;

    public static Servo GearServo;
    public static Servo AirplaneServo;
    public static Servo IntakeServo;

    // Position for the Claws to close

    public static double Open = 0.33;
    public static double Close = 0;

    public static double Raise = .33;
    public static final int slideySlideMax = 7700;
    public static final int slideySlideMin = 100;

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


    public EngiNERDs_Variables(HardwareMap hardwareMap) {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLiftyLift = hardwareMap.dcMotor.get("motorLiftyLift");
        motorRiseyRise = hardwareMap.dcMotor.get("motorRiseyRise");

        LeftClaw = hardwareMap.servo.get("LeftClaw");
        RightClaw = hardwareMap.servo.get("RightClaw");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        GearServo = hardwareMap.servo.get("GearServo");
        AirplaneServo = hardwareMap.servo.get("AirplaneServo");


        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorRiseyRise.setPower(0);
        motorLiftyLift.setPower(0);

        // this call sets the servos during initialization
        LeftClaw.setPosition(1);
        RightClaw.setPosition(0);
        AirplaneServo.setPosition(1);

        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftyLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorRiseyRise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftyLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLiftyLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRiseyRise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Setting the motor Direction, so the motors rotate correctly (Default Direction = Forward)
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftyLift.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftClaw.setDirection(Servo.Direction.REVERSE);
        RightClaw.setDirection(Servo.Direction.REVERSE);
        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);
        GearServo.setDirection(Servo.Direction.REVERSE);

    }

    }
