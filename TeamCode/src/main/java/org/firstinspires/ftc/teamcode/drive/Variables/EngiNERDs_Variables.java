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
    public static Servo WristServo;

    public static Servo AirplaneLaunchServo;

    public static RevBlinkinLedDriver LEDs;

    ///////////////////////////////////////////////////////////////////////////////////////
    // Proper Rotation Variables ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    public static double ShootPlane = 90;
    public static double AirplaneMount_Rotate = 60;
    public static final int slideySlideMax = 7700;
    public static final int slideySlideMin = 100;

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
        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");
        WristServo = hardwareMap.servo.get("WristServo");

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
        WristServo.setDirection(Servo.Direction.FORWARD);
        FlooppyFloop.setDirection(Servo.Direction.FORWARD);
        AirplaneLaunchServo.setDirection(Servo.Direction.REVERSE);

    }
}
