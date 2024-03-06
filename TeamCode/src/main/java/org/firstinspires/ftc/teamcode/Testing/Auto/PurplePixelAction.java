package org.firstinspires.ftc.teamcode.Testing.Auto;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ClosePixelCover;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DropPurplePixel;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Wrist_Init_AutoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Wrist_Init_AutoR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.initFlip;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tuning_Variables.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tuning_Variables.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", preselectTeleOp = "EngiNERDs_Control_RC_V2")
//@Disabled
public class PurplePixelAction extends LinearOpMode {

    public Servo PurplePixelServo;
    public Servo AirplaneLaunchServo;
    public Servo PixelCoverServo;
    public Servo FlooppyFloop;
    public Servo FlippyFlip;
    public Servo WristServoR;
    public Servo WristServoL;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        PurplePixelServo = hardwareMap.servo.get("PurplePixelServo");
        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");
        PixelCoverServo = hardwareMap.servo.get("PixelCoverServo");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        WristServoR = hardwareMap.servo.get("WristServoR");
        WristServoL = hardwareMap.servo.get("WristServoL");

        PixelCoverServo.setPosition(ClosePixelCover * DegreeTorque);
        PurplePixelServo.setPosition(init * DegreeTorque);
        AirplaneLaunchServo.setPosition(init * DegreeTorque);
        FlooppyFloop.setPosition(init * Degree5Turn);
        FlippyFlip.setPosition(initFlip * Degree5Turn);
        WristServoR.setPosition(Wrist_Init_AutoR * Degree5Turn);
        WristServoL.setPosition(Wrist_Init_AutoL * Degree5Turn);

        FlippyFlip.setDirection(Servo.Direction.FORWARD);
        PixelCoverServo.setDirection(Servo.Direction.FORWARD);
        FlooppyFloop.setDirection(Servo.Direction.FORWARD);
        WristServoL.setDirection(Servo.Direction.FORWARD);
        WristServoR.setDirection(Servo.Direction.REVERSE);
        AirplaneLaunchServo.setDirection(Servo.Direction.REVERSE);
        PurplePixelServo.setDirection(Servo.Direction.REVERSE);


        TrajectorySequence tajpark = drive.trajectorySequenceBuilder(new Pose2d())

                .back(30)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                PurplePixelServo.setPosition(DropPurplePixel * DegreeTorque);
                })
                .forward(20)
                .build();


        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySequence(tajpark);

        sleep(20000);
    }
}
