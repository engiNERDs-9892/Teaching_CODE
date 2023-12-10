package org.firstinspires.ftc.teamcode.autos.Parking;

import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.RightClaw;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
//@Disabled
@Autonomous(group = "drive")
public class Auto_Blue_Park_Far extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        GearServo = hardwareMap.servo.get("GearServo");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        LeftClaw = hardwareMap.servo.get("LeftClaw");
        RightClaw = hardwareMap.servo.get("RightClaw");

        FlooppyFloop.setPosition(.85);
        FlippyFlip.setPosition(.15);
        GearServo.setPosition(.5);
        LeftClaw.setPosition(1);
        RightClaw.setPosition(0);


        LeftClaw.setDirection(Servo.Direction.REVERSE);
        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);
        GearServo.setDirection(Servo.Direction.REVERSE);

            Pose2d startPoseBlueRight = new Pose2d(-36, 60, Math.toRadians(180.00));

            drive.setPoseEstimate(startPoseBlueRight);


        TrajectorySequence bluerightpark = drive.trajectorySequenceBuilder(startPoseBlueRight)
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    FlooppyFloop.setPosition(0.10);
                    FlippyFlip.setPosition(0.90);
                })


                .lineToConstantHeading(new Vector2d(-36,57))
                .lineToConstantHeading(new Vector2d(30,57))
                .lineToConstantHeading(new Vector2d(30,10))
                .lineToConstantHeading(new Vector2d(60,10))

                .build();

            waitForStart();

            if (!isStopRequested())
                drive.followTrajectorySequence(bluerightpark);



    }
    }
