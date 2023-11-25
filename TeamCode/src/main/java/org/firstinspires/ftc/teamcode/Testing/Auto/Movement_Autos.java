package org.firstinspires.ftc.teamcode.Testing.Auto;

import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.Open;
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
public class Movement_Autos extends LinearOpMode {
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


            Pose2d startPoseBlueLeft = new Pose2d(12, 60, Math.toRadians(-90.00));
            Pose2d startPoseBlueRight = new Pose2d(-36, 60, Math.toRadians(-90.00));
            Pose2d startPoseRedLeft = new Pose2d(-36, -60, Math.toRadians(90.00));
            Pose2d startPoseRedRight = new Pose2d(12, -60, Math.toRadians(90.00));

            drive.setPoseEstimate(startPoseBlueLeft);
            drive.setPoseEstimate(startPoseBlueRight);
            drive.setPoseEstimate(startPoseRedLeft);
            drive.setPoseEstimate(startPoseRedRight);


        TrajectorySequence redrightR = drive.trajectorySequenceBuilder(startPoseRedRight)
                .lineToConstantHeading(new Vector2d(25, -40))

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(50, -40, Math.toRadians(0)), Math.toRadians(0.00))

                .build();

            waitForStart();

            if (!isStopRequested())
                drive.followTrajectorySequence(redrightR);



    }
    }
