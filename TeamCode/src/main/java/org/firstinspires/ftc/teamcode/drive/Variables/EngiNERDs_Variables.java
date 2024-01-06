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

    public static Servo AirplaneMountServo;
    public static Servo IntakeServo;
    public static Servo AirplaneLaunchServo;

    // Position for the Claws to close
    public static double Open = 45;
    public static double Close = 0;
    public static double AirplaneMount_Rotate = 60;
    public static final int slideySlideMax = 7700;
    public static final int slideySlideMin = 100;


    public static double DegreeClaw = 0.00333333333;  // 1/300 (Torque Servo)
    public static double DegreeArm = 0.00333333333; //  1/300 (Torque Servo)
    public static double DegreeAirplane = 0.00333333333; //  1/300 (Torque Servo)
    public static double DegreeWrist = 0.000555555556; // 1/1800 (5 Turn Servo)

    public static double GroundArmRotateL = 9;
    public static double GroundArmRotateR = 9;
    public static double BackboardArmRotate = 175;


    ///////////////////////////////////////////////////////////////////
    // *NOTE* that 3.5 Degrees = Have Inch in movement for the Wrist //
    ///////////////////////////////////////////////////////////////////
    public static double Stack5ArmRotate = 23;

    public static double Stack4ArmRotate = 19.5;

    public static double Stack3ArmRotate = 16;

    public static double Stack2ArmRotate = 12.5;

    public static double Stack1ArmRotate = 8;

    public static double WristRotateGround = 0;
    public static double WristRotateStack = 0;
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

        // this sets the servos in the proper direction
        FlippyFlip.setDirection(Servo.Direction.FORWARD);
        RightClaw.setDirection(Servo.Direction.FORWARD);
        GearServo.setDirection(Servo.Direction.FORWARD);
        AirplaneMountServo.setDirection(Servo.Direction.FORWARD);
        LeftClaw.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);

        AirplaneMountServo.setPosition(0 * DegreeAirplane);
        LeftClaw.setPosition(0 * DegreeClaw); // Closes
        RightClaw.setPosition(0 * DegreeClaw); // Closes
    }
}
