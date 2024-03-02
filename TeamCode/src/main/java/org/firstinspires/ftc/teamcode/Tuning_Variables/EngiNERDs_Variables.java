package org.firstinspires.ftc.teamcode.Tuning_Variables;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class EngiNERDs_Variables {
    public static DcMotor motorFL = null;
    public static DcMotor motorBL = null;
    public static DcMotor motorFR = null;
    public static DcMotor motorBR = null;

    public static DcMotor motorLiftyLift;

    public static DcMotor motorRiseyRise;
    public static DcMotor motorINTAKE;

    public static Servo FlippyFlip; // Arm Rotation Servo Left

    public static Servo FlooppyFloop; // Arm Rotation Servo Right
    public static Servo WristServoL; // Wrist Rotation Servo Left
    public static Servo WristServoR; // Wrist Rotation Servo Right

    public static Servo AirplaneLaunchServo;
    public static Servo PurplePixelServo;

    public static RevBlinkinLedDriver LEDs;

    ///////////////////////////////////////////////////////////////////////////////////////
    // Proper Rotation Variables ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    public static double InitPurplePixel = 0;
    public static double InitArms = 0;
    public static double InitWrist = 0;
    public static double InitPlane = 0;
    public static double DropPurplePixel = 94.5;
    public static double LaunchPlane = 67.5;
    public static double ShootPlane = 90;
    public static double DegreeTorque = 0.00333333333;  // 1/300 (Torque Servo)
    public static double Degree5Turn = 0.000555555556; // 1/1800 (5 Turn Servo)




    public EngiNERDs_Variables(HardwareMap hardwareMap) {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLiftyLift = hardwareMap.dcMotor.get("motorLiftyLift");
        motorRiseyRise = hardwareMap.dcMotor.get("motorRiseyRise");

        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        WristServoL = hardwareMap.servo.get("WristServoL");
        WristServoR = hardwareMap.servo.get("WristServoR");
        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");
        PurplePixelServo = hardwareMap.servo.get("PurplePixelServo");

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
        motorINTAKE.setDirection(DcMotorSimple.Direction.FORWARD);

        // this sets the servos in the proper direction
        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.FORWARD);
        WristServoL.setDirection(Servo.Direction.FORWARD);
        WristServoR.setDirection(Servo.Direction.FORWARD);
        AirplaneLaunchServo.setDirection(Servo.Direction.REVERSE);
        PurplePixelServo.setDirection(Servo.Direction.REVERSE);

    }
}
