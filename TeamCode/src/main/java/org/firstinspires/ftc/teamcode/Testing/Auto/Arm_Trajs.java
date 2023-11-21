package org.firstinspires.ftc.teamcode.Testing.Auto;

import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.Open;
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
public class Arm_Trajs extends LinearOpMode {
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

        TrajectorySequence DroponBack = drive.trajectorySequenceBuilder(startPose)

                .addDisplacementMarker(() -> {
                    FlooppyFloop.setPosition(.15);
                    FlippyFlip.setPosition(.85);
                    GearServo.setPosition(.6);
                    sleep(1000);
                    RightClaw.setPosition(Open);
                })
                .build();

        TrajectorySequence PixelDrop =  drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(() -> {
                    FlooppyFloop.setPosition(.03);
                    FlippyFlip.setPosition(.97);
                    sleep(500);
                    GearServo.setPosition(.85);
                    sleep(1000);
                    LeftClaw.setPosition(Open);
                })
                        .build();

            waitForStart();

            if (!isStopRequested())
                drive.followTrajectorySequence(PixelDrop);



    }
    }
