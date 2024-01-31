package org.firstinspires.ftc.teamcode.drive.Variables;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
    public static DcMotor motorINTAKE;

    public static Servo FlippyFlip;

    public static Servo FlooppyFloop;

    public static Servo LeftClaw;

    public static Servo RightClaw;

    public static Servo GearServo;

    public static Servo AirplaneMountServo;
    public static Servo IntakeServo;
    public static Servo AirplaneLaunchServo;

    public static RevBlinkinLedDriver LEDs;

    // Position for the Claws to close
    public static double Open = 45;
    public static double OpenAuto = 65;
    public static double Close = 0;
    public static double ShootPlane = 90;
    public static double AirplaneMount_Rotate = 60;
    public static final int slideySlideMax = 7700;
    public static final int slideySlideMin = 100;


    public static double DegreeClaw = 0.00333333333;  // 1/300 (Torque Servo)
    public static double DegreeArm = 0.000555555556; //  1/300 (Torque Servo)
    public static double DegreeAirplane = 0.00333333333; //  1/300 (Torque Servo)
    public static double DegreeWrist = 0.000555555556; // 1/1800 (5 Turn Servo)

    public static double GroundArmRotate = 3;
    public static double FrontScoringArmRotate = 10; // WIP
    public static double FrontScoringArmRotat= 8; // WIP
    public static double FrontScoringArmRotateFlip = 8;
    public static double BackboardArmRotate = 170;
    public static double BackboardArmRotat = 175;
    public static double SafeRotateBackboard = 180;


    ///////////////////////////////////////////////////////////////////
    // *NOTE* that 3.5 Degrees = Have Inch in movement for the Wrist //
    ///////////////////////////////////////////////////////////////////
    public static double InitArmRotateFloop = 1775;
    public static double InitArmRotateFlip = 31;

    public static double Stack5ArmRotateFloop = 1769;
    public static double Stack5ArmRotateFlip = 36;

    public static double Stack4ArmRotateFloop = 1773.5;
    public static double Stack4ArmRotateFlip = 31.5;

    public static double Stack3ArmRotateFloop = 1779;
    public static double Stack3ArmRotateFlip = 28;

    public static double Stack2ArmRotateFloop = 1783;
    public static double Stack2ArmRotateFlip = 24;

    public static double Stack1ArmRotateFloop = 1786;
    public static double Stack1ArmRotateFlip = 20;

    public static double RESET_ARM = 13.5;
    public static double RESETARM = 18.5;

    public static double WristRotateInit = 225;
    public static double WristRotateGround = 0;
    public static double WristRotateStack = 0;
    public static double WristRotateFrontScoring = 95; // WIP
    public static double WristRotateBackboard = 270;


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
        AirplaneMountServo = hardwareMap.servo.get("AirplaneMountServo");
        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");

        LEDs = hardwareMap.get(RevBlinkinLedDriver.class,"LEDs");





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
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLiftyLift.setDirection(DcMotorSimple.Direction.FORWARD);

        // this sets the servos in the proper direction
        FlippyFlip.setDirection(Servo.Direction.FORWARD);
        RightClaw.setDirection(Servo.Direction.FORWARD);
        GearServo.setDirection(Servo.Direction.FORWARD);
        AirplaneMountServo.setDirection(Servo.Direction.FORWARD);
        FlooppyFloop.setDirection(Servo.Direction.FORWARD);
        LeftClaw.setDirection(Servo.Direction.REVERSE);


        AirplaneMountServo.setPosition(0 * DegreeAirplane);
        LeftClaw.setPosition(0 * DegreeClaw); // Closes
        RightClaw.setPosition(0 * DegreeClaw); // Closes
    }
}
