package org.firstinspires.ftc.teamcode.autos.Parking;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous(group = "drive")
public class Auto_Red_Park_Far extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        GearServo = hardwareMap.servo.get("GearServo");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        LeftClaw = hardwareMap.servo.get("LeftClaw");
        RightClaw = hardwareMap.servo.get("RightClaw");
        AirplaneServo = hardwareMap.servo.get("AirplaneServo");


        // this call sets the servos during initialization
        FlooppyFloop.setPosition(.85);
        FlippyFlip.setPosition(.15);
        GearServo.setPosition(.7);
        LeftClaw.setPosition(0);
        RightClaw.setPosition(1);
        AirplaneServo.setPosition(1);

        // this sets the servos in the proper direction
        LeftClaw.setDirection(Servo.Direction.REVERSE);
        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);
        GearServo.setDirection(Servo.Direction.REVERSE);

            Pose2d startPoseRedLeft = new Pose2d(-36, -60, Math.toRadians(180.00));

            drive.setPoseEstimate(startPoseRedLeft);


        TrajectorySequence redleftpark = drive.trajectorySequenceBuilder(startPoseRedLeft)
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    FlooppyFloop.setPosition(0.10);
                    FlippyFlip.setPosition(0.90);
                })

                .lineToConstantHeading(new Vector2d(-36,-57))
                .lineToConstantHeading(new Vector2d(30,-57))
                .lineToConstantHeading(new Vector2d(30,-10))
                .lineToConstantHeading(new Vector2d(60,-10))
                .build();

            waitForStart();

            if (!isStopRequested())
                drive.followTrajectorySequence(redleftpark);



    }
    }
