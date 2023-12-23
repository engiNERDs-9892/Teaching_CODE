package org.firstinspires.ftc.teamcode.Testing.Auto;

import static org.firstinspires.ftc.teamcode.drive.Variables.Autonomous_Variables.AirplaneServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.Autonomous_Variables.HookL;
import static org.firstinspires.ftc.teamcode.drive.Variables.Autonomous_Variables.HookR;
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Variables.Autonomous_Variables;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous(group = "drive")
public class Testing_Autos extends LinearOpMode {



    @Override
    public void runOpMode() {

        Autonomous_Variables.GearServo = hardwareMap.servo.get("GearServo");
        Autonomous_Variables.FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        Autonomous_Variables.FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        Autonomous_Variables.LeftClaw = hardwareMap.servo.get("LeftClaw");
        Autonomous_Variables.RightClaw = hardwareMap.servo.get("RightClaw");
        AirplaneServo = hardwareMap.servo.get("AirplaneServo");
        HookR = hardwareMap.servo.get("HookR");
        HookL = hardwareMap.servo.get("HookL");

        // this call sets the servos during initialization
        Autonomous_Variables.LeftClaw.setPosition(0);
        Autonomous_Variables.RightClaw.setPosition(1);
        HookR.setPosition(1);
        HookL.setPosition(1);
        AirplaneServo.setPosition(1);
        Autonomous_Variables.GearServo.setPosition(0.5);
        Autonomous_Variables.FlooppyFloop.setPosition(.85);
        Autonomous_Variables.FlippyFlip.setPosition(.15);

        Autonomous_Variables.FlippyFlip.setDirection(Servo.Direction.REVERSE);
        Autonomous_Variables.FlooppyFloop.setDirection(Servo.Direction.REVERSE);
        Autonomous_Variables.LeftClaw.setDirection(Servo.Direction.REVERSE);
        Autonomous_Variables.GearServo.setDirection(Servo.Direction.REVERSE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPoseBlueLeft = new Pose2d(12, 60, Math.toRadians(-90.00));
        Pose2d startPoseBlueRight = new Pose2d(-36, 60, Math.toRadians(-90.00));
        Pose2d startPoseRedLeft = new Pose2d(-36, -60, Math.toRadians(90.00));
        Pose2d startPoseRedRight = new Pose2d(12, -60, Math.toRadians(90.00));

        drive.setPoseEstimate(startPoseBlueLeft);
        drive.setPoseEstimate(startPoseBlueRight);
        drive.setPoseEstimate(startPoseRedLeft);
        drive.setPoseEstimate(startPoseRedRight);


        // Red Left
        TrajectorySequence redrightL = drive.trajectorySequenceBuilder(startPoseRedRight)
                .lineToLinearHeading(new Pose2d(26, 60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(26, 10, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(26, 40, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(53, 40, Math.toRadians(0)))

                .build();

        TrajectorySequence redrightM = drive.trajectorySequenceBuilder(startPoseRedRight)
                // Knock the Team Prop out of the way
                .lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(90)))

                // Place the purple Pixel
                .lineToLinearHeading(new Pose2d(12, -37, Math.toRadians(90)))
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    FlooppyFloop.setPosition(0.03);
                    FlippyFlip.setPosition(0.97);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    GearServo.setPosition(.98);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    LeftClaw.setPosition(Open);
                })


                // Flip the arm to the backboard
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    FlooppyFloop.setPosition(.15);
                    FlippyFlip.setPosition(.85);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                    GearServo.setPosition(.75);
                })


                // Place the Orange Pixel
                .lineToLinearHeading(new Pose2d(52, -36, Math.toRadians(0)))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    RightClaw.setPosition(Open);
                })

                .build();


        TrajectorySequence redrightR = drive.trajectorySequenceBuilder(startPoseRedRight)
                .splineToLinearHeading(new Pose2d(9, -40, Math.toRadians(180.00)), Math.toRadians(100))
                .lineToConstantHeading(new Vector2d(8, -32.5))
                .lineToConstantHeading(new Vector2d(0, -32.5))
                .lineToConstantHeading(new Vector2d(8, -30))
                .lineToLinearHeading(new Pose2d(53, -18, Math.toRadians(0)))
                .build();


        // Blue Left
        TrajectorySequence blueleftL = drive.trajectorySequenceBuilder(startPoseBlueLeft)
                .lineToLinearHeading(new Pose2d(26, 40, Math.toRadians(-92)))
                .lineToLinearHeading(new Pose2d(53, 40, Math.toRadians(-2)))
                .build();

        TrajectorySequence blueleftM = drive.trajectorySequenceBuilder(startPoseBlueLeft)
                .lineToLinearHeading(new Pose2d(12, 30, Math.toRadians(-92)))
                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(-92)))
                .lineToLinearHeading(new Pose2d(53.3, 36, Math.toRadians(-2)))

                .build();

        TrajectorySequence blueleftR = drive.trajectorySequenceBuilder(startPoseBlueLeft)
                .splineToLinearHeading(new Pose2d(9, 40, Math.toRadians(180.00)), Math.toRadians(-100))
                .lineToConstantHeading(new Vector2d(8, 32.5))
                .lineToConstantHeading(new Vector2d(0, 32.5))
                .lineToConstantHeading(new Vector2d(8, 30))
                .lineToLinearHeading(new Pose2d(53, 40, Math.toRadians(0)))
                .build();


        // Red Left
        TrajectorySequence redleftR = drive.trajectorySequenceBuilder(startPoseRedLeft)
                // Drop purple pixel
                .splineToLinearHeading(new Pose2d(-33, -40, Math.toRadians(0)), Math.toRadians(100))
                .lineToConstantHeading(new Vector2d(-32, -32.5))
                .lineToConstantHeading(new Vector2d(-24, -32.5))
                .lineToConstantHeading(new Vector2d(-32, -30))


                // Play orange pixel
                .lineToLinearHeading(new Pose2d(-37, -8, Math.toRadians(4)))
                .lineToConstantHeading(new Vector2d(20, -10))
                .splineToConstantHeading(new Vector2d(52, -40), Math.toRadians(0))

                .build();

        TrajectorySequence redleftM = drive.trajectorySequenceBuilder(startPoseRedLeft)
                .lineToLinearHeading(new Pose2d(-36, -10, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-37, -10, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(20, -10))
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(53.3, -36))
                .build();

        TrajectorySequence redleftL = drive.trajectorySequenceBuilder(startPoseRedLeft)
                .lineToLinearHeading(new Pose2d(-47, -60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-47, -10, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-47, -40, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(2)))
                .lineToConstantHeading(new Vector2d(20, -12))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(53, -35, Math.toRadians(0)), Math.toRadians(0))
                .build();


        // Blue Right
        TrajectorySequence bluerightR = drive.trajectorySequenceBuilder(startPoseBlueRight)
                .lineToLinearHeading(new Pose2d(26, -40, Math.toRadians(92)))
                .lineToLinearHeading(new Pose2d(53, -40, Math.toRadians(2)))
                .strafeRight(10)

                .build();

        TrajectorySequence bluerightM = drive.trajectorySequenceBuilder(startPoseBlueRight)
                .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(92)))
                .lineToLinearHeading(new Pose2d(53, -35, Math.toRadians(2)))
                .strafeRight(30)

                .build();

        TrajectorySequence bluerightL = drive.trajectorySequenceBuilder(startPoseBlueRight)
                .splineToLinearHeading(new Pose2d(9, -40, Math.toRadians(180.00)), Math.toRadians(100))
                .lineToConstantHeading(new Vector2d(8, -32.5))
                .lineToConstantHeading(new Vector2d(0, -32.5))
                .lineToConstantHeading(new Vector2d(8, -30))
                .lineToLinearHeading(new Pose2d(53, -22, Math.toRadians(0)))
                .build();


        waitForStart();


        while (opModeIsActive()) {

            if (!isStopRequested())
                drive.followTrajectorySequence(redrightM);


        }
    }
}
