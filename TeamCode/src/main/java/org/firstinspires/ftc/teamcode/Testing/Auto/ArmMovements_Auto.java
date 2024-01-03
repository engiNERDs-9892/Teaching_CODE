package org.firstinspires.ftc.teamcode.Testing.Auto;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.BackboardArmRotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GroundArmRotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack5ArmRotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateBackboard;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateGround;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateStack;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeArm;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeWrist;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Examples.RedPipline;
import org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// @ CONFIG is used for FTC Dashboard
@Config
//@Disabled
@Autonomous(group = "drive")
public class ArmMovements_Auto extends LinearOpMode {

    // Calls the Variable webcam
    OpenCvWebcam webcam;
    // Calls the proper pipline in order to detect the correct color (in this case its red)
    RedPipline.redPipline pipeline;
    // This just is determining the default position of the camera detection (this is right due to where our camera is placed)
    RedPipline.redPipline.Detection_Positions snapshotAnalysis = RedPipline.redPipline.Detection_Positions.RIGHT; // default

    @Override
    public void runOpMode() {

        // This calls the hardware map for servos and motors
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        new EngiNERDs_Variables(hardwareMap);

        // this call sets the servos during initialization
        FlooppyFloop.setPosition(50 * DegreeArm); // Rotates at an angle forwards
        FlippyFlip.setPosition(45 * DegreeArm); // rotates at an angle forwards
        GearServo.setPosition(225 * DegreeWrist); // Rotates into the air


        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RedPipline.redPipline();
        webcam.setPipeline(pipeline);

        // This is so we can view what the camera is seeing
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // This is in what viewing window the camera is seeing through and it doesn't matter
                // what orientation it is | UPRIGHT, SIDEWAYS_LEFT, SIDEWAYS_RIGHT, etc.

                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();


        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        // Give the robot a starting position on the coordinate plane
        Pose2d startPoseRedRight = new Pose2d(12, -60, Math.toRadians(90.00));
        drive.setPoseEstimate(startPoseRedRight);


        //
        TrajectorySequence PurplePixelArmMovement = drive.trajectorySequenceBuilder(startPoseRedRight)

                .forward(1)

                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    FlooppyFloop.setPosition(GroundArmRotate * DegreeArm);
                    FlippyFlip.setPosition(GroundArmRotate * DegreeArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    RightClaw.setPosition(Open * DegreeClaw);
                })
                .waitSeconds(25)

                .build();

        //
        TrajectorySequence OrangePixelArmMovement = drive.trajectorySequenceBuilder(startPoseRedRight)

                .forward(1)

                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    FlooppyFloop.setPosition(BackboardArmRotate * DegreeArm);
                    FlippyFlip.setPosition(BackboardArmRotate * DegreeArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    GearServo.setPosition(WristRotateBackboard * DegreeWrist);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    LeftClaw.setPosition(Open * DegreeClaw);
                })
                .waitSeconds(25)

                .build();



        //
        TrajectorySequence WhitePixelArmMovement = drive.trajectorySequenceBuilder(startPoseRedRight)

                .forward(1)

                .UNSTABLE_addTemporalMarkerOffset(-1.25, () -> {
                    FlooppyFloop.setPosition(Stack5ArmRotate * DegreeArm);
                    FlippyFlip.setPosition(Stack5ArmRotate * DegreeArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    GearServo.setPosition(WristRotateStack * DegreeWrist);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    RightClaw.setPosition(Open  * DegreeClaw);
                })
                .waitSeconds(25)

                .build();


        // This is starting after the driver presses play
        waitForStart();


        drive.followTrajectorySequence(OrangePixelArmMovement);
        }
    }