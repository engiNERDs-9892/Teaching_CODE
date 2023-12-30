package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.BackboardArmLRotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.BackboardArmRRotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.ClawL_Close;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.ClawL_Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.ClawR_Close;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.ClawR_Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GroundArmLRotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GroundArmRRotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateBackboard;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateGround;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Examples.RedPipline;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//////////////////////////////////////////////
// AUTO PLAN - This is as of Dec 23, 2023    //
//                                          //
// a = Right, b = Middle, c = Left          //
//////////////////////////////////////////////

// 1. Drive to the location of the Team Prop and move the team prop out of the way
// 1a. Drive Diagonal Left and Forwards to move the team prop into the middle of the trusses
// 1b. Spline at a 180 Degree (Robot facing the Truss) Forward enough to place the purple pixel without driving into the Team prop
// 1c.

// 2. Place the purple pixel depending on wherever the Team Prop is located
// 2a.
// 2b. Drop the Purple Pixel
// 2c.

// 3. Place the Orange Pixel wherever the Team prop is located
// 3a.
// 3b. Drive spline backwards at the same heading and play the pixel behind us
// 3c.

// 4. Drive to pick up 2 white pixels (Ideally)


// @ CONFIG is used for FTC Dashboard
@Config
@Disabled
@Autonomous(group = "drive")
public class Red_Auto extends LinearOpMode {

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
        LeftClaw.setPosition(1);
        RightClaw.setPosition(0);
        AirplaneServo.setPosition(1);

        // this sets the servos in the proper direction
        LeftClaw.setDirection(Servo.Direction.REVERSE);
        RightClaw.setDirection(Servo.Direction.REVERSE);
        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);
        GearServo.setDirection(Servo.Direction.REVERSE);


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


        // This is if the camera detects the left side (code of what it does is below)
        TrajectorySequence redrightL = drive.trajectorySequenceBuilder(startPoseRedRight)

                // Knock the Team Prop out of the way
                .splineToLinearHeading(new Pose2d(9, -40, Math.toRadians(180.00)), Math.toRadians(100))
                .lineToConstantHeading(new Vector2d(8, -32.5))
                .lineToConstantHeading(new Vector2d(0, -32.5))

                // Place the Purple Pixel
                .lineToConstantHeading(new Vector2d(9, -30))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    FlooppyFloop.setPosition(GroundArmLRotate);
                    FlippyFlip.setPosition(GroundArmRRotate);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    GearServo.setPosition(WristRotateGround);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    RightClaw.setPosition(ClawR_Open);
                })


                // Raise for the backboard
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    FlooppyFloop.setPosition(.15);
                    FlippyFlip.setPosition(.85);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    GearServo.setPosition(.75);
                })


                // Place the Orange Pixel
                .lineToLinearHeading(new Pose2d(52, -15.75, Math.toRadians(0)))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    LeftClaw.setPosition(Open);
                })

                .lineToLinearHeading(new Pose2d(53, -52, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(48, -68, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(62, -68, Math.toRadians(180)))

                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    FlooppyFloop.setPosition(0.03);
                    FlippyFlip.setPosition(0.97);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    GearServo.setPosition(.98);
                })
                .build();

        // This is if the camera detects the middle (code of what it does is below)
        TrajectorySequence redrightM = drive.trajectorySequenceBuilder(startPoseRedRight)

                /////////////////////////////////////////////////////////////////////////
                // 2.35 Seconds to drive to the Spike Mark (PURPLE PIXEL | RIGHT CLAW) //
                /////////////////////////////////////////////////////////////////////////

                .splineToLinearHeading(new Pose2d(16.00, -37.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                .setReversed(true)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    FlooppyFloop.setPosition(GroundArmLRotate);
                    FlippyFlip.setPosition(GroundArmRRotate);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    GearServo.setPosition(WristRotateGround);
                })

                // Left Claw (PURPLE PIXEL)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    RightClaw.setPosition(ClawR_Open);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    RightClaw.setPosition(ClawR_Close);
                })


                /////////////////////////////////////////////////////////////////////////
                // 2.00 Seconds to drive to the Backboard (ORANGE PIXEL | RIGHT CLAW)  //
                /////////////////////////////////////////////////////////////////////////

                .UNSTABLE_addTemporalMarkerOffset(.65, () -> {
                    GearServo.setPosition(WristRotateBackboard);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    FlooppyFloop.setPosition(BackboardArmLRotate);
                    FlippyFlip.setPosition(BackboardArmRRotate);
                })

                .splineToLinearHeading(new Pose2d(51, -42, Math.toRadians(-180.00)), Math.toRadians(0.00))
                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    LeftClaw.setPosition(ClawL_Open);
                })

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    LeftClaw.setPosition(ClawL_Close);
                })


                ////////////////////////////////////////////////////////////////////////////////////
                // 1.79 Seconds to drive to Park (RESET CLAWS / WRIST - *POSSIBLY EXTEND INTAKE)  //
                ////////////////////////////////////////////////////////////////////////////////////
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(57, -68, Math.toRadians(-180.00)), Math.toRadians(-90))
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    GearServo.setPosition(WristRotateGround);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    FlooppyFloop.setPosition(GroundArmLRotate);
                    FlippyFlip.setPosition(GroundArmRRotate);
                })


                .build();


        // This is if the camera detects the right side (code of what it does is below)
        TrajectorySequence redrightR = drive.trajectorySequenceBuilder(startPoseRedRight)

                /////////////////////////////////////////////////////////////////////////
                // 1.70 Seconds to drive to the Spike Mark (PURPLE PIXEL | RIGHT CLAW) //
                /////////////////////////////////////////////////////////////////////////
                .lineToLinearHeading(new Pose2d(25, -43, Math.toRadians(90)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-1.1, () -> {
                    FlooppyFloop.setPosition(GroundArmLRotate);
                    FlippyFlip.setPosition(GroundArmRRotate);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.75, () -> {
                    GearServo.setPosition(WristRotateGround);
                })

                // Right Claws (PURPLE PIXEL)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    RightClaw.setPosition(ClawR_Open);
                })

                /////////////////////////////////////////////////////////////////////////
                // 2.40 Seconds to drive to the Backboard (ORANGE PIXEL | RIGHT CLAW)  //
                /////////////////////////////////////////////////////////////////////////

                // START MOVING TO THE BACKBOARD STARTS NOW
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    RightClaw.setPosition(ClawR_Close);
                })

                // Rotate to Backboard Position
                .UNSTABLE_addTemporalMarkerOffset(.65, () -> {
                    GearServo.setPosition(WristRotateBackboard);
                })

                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    FlooppyFloop.setPosition(BackboardArmLRotate);
                    FlippyFlip.setPosition(BackboardArmRRotate);
                })
                .lineToLinearHeading(new Pose2d(53, -45, Math.toRadians(180)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    LeftClaw.setPosition(ClawL_Open);
                })



                ////////////////////////////////////////////////////////////////////////////////////
                // 1.66 Seconds to drive to Park (RESET CLAWS / WRIST - *POSSIBLY EXTEND INTAKE)  //
                ////////////////////////////////////////////////////////////////////////////////////
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(45, -55, Math.toRadians(180.00)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(68, -68, Math.toRadians(180.00)), Math.toRadians(-90))
                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    LeftClaw.setPosition(ClawL_Close);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    FlooppyFloop.setPosition(GroundArmLRotate);
                    FlippyFlip.setPosition(GroundArmRRotate);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    GearServo.setPosition(WristRotateGround);
                })


                .build();


        // This is starting after the driver presses play
        waitForStart();


        switch (snapshotAnalysis) {
            case LEFT: // Level 3
            {
                drive.followTrajectorySequence(redrightL);


                break;

            }


            case RIGHT: // Level 1
            {

                drive.followTrajectorySequence(redrightR);


                break;
            }

            case CENTER: // Level 2
            {

                drive.followTrajectorySequence(redrightM);

                break;
            }


        }
    }
}

