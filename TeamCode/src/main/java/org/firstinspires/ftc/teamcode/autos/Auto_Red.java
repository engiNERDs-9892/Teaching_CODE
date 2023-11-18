package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.RightClaw;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
//@Disabled
@Autonomous(group = "drive")
public class Auto_Red extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        GearServo = hardwareMap.servo.get("GearServo");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        LeftClaw = hardwareMap.servo.get("LeftClaw");
        RightClaw = hardwareMap.servo.get("RightClaw");

        FlooppyFloop.setPosition(.7);
        FlippyFlip.setPosition(.3);
        GearServo.setPosition(.2);
        LeftClaw.setPosition(1);
        RightClaw.setPosition(0);


        LeftClaw.setDirection(Servo.Direction.REVERSE);
        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);
        GearServo.setDirection(Servo.Direction.REVERSE);

            Pose2d startPose = new Pose2d(0, 0, 0);

            drive.setPoseEstimate(startPose);

            TrajectorySequence redleftL = drive.trajectorySequenceBuilder(startPose)
                    .forward(23)
                    .turn(Math.toRadians(92))
                    .forward(7)
                    .addTemporalMarker(() -> {
                        FlooppyFloop.setPosition(.03);
                        FlippyFlip.setPosition(.97);
                        sleep(1000);
                        GearServo.setPosition(.85);
                    })
                    .waitSeconds(3)
                    .back(40)
                    .turn(Math.toRadians(184))
                    .waitSeconds(10000)
                    // Drop Orange
                    .build();

        TrajectorySequence redleftM = drive.trajectorySequenceBuilder(startPose)
                .forward(23)
                .turn(Math.toRadians(-92))
                .build();


        TrajectorySequence redleftR = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(12)
                .forward(40)
                .back(25)
                .turn(Math.toRadians(10))
                .addTemporalMarker(() -> {
                    FlooppyFloop.setPosition(.03);
                    FlippyFlip.setPosition(.97);
                    sleep(1000);
                    GearServo.setPosition(.85);
                    LeftClaw.setPosition(.33);
                })
                .waitSeconds(3)
                // Drop Purple Pixel
                .addTemporalMarker(() -> {
                    FlooppyFloop.setPosition(.2);
                    FlippyFlip.setPosition(.88);
                    GearServo.setPosition(.6);

                })
                .turn(Math.toRadians(-98))
                .forward(23)
                .addTemporalMarker(() -> {
                    RightClaw.setPosition(.33);
                })

                .back(4)
                .strafeRight(18)
                .build();

            waitForStart();

            if (!isStopRequested())
                drive.followTrajectorySequence(redleftR);



    }
    }
