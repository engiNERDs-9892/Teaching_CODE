package org.firstinspires.ftc.teamcode.drive.Variables;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class TeleOP_Variables {

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
    public static Servo HookL;
    public static Servo HookR;
    public static Servo IntakeServo;

    // Position for the Claws to close

    public static double Open = 0.33;
    public static double Close = 0;

    public static double Raise = .33;
    public static final int slideySlideMax = 7700;
    public static final int slideySlideMin = 100;


    public TeleOP_Variables(HardwareMap hardwareMap) {
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
        HookR = hardwareMap.servo.get("HookR");
        HookL = hardwareMap.servo.get("HookL");


        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorRiseyRise.setPower(0);
        motorLiftyLift.setPower(0);

        // this call sets the servos during initialization
        LeftClaw.setPosition(1);
        RightClaw.setPosition(0);
        HookR.setPosition(1);
        HookL.setPosition(1);
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
