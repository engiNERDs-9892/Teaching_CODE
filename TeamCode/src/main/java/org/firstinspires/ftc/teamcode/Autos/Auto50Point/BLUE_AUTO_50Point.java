package org.firstinspires.ftc.teamcode.Autos.Auto50Point;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.BackboardAutoArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.BackboardAutoArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.BackboardAutoWristBack;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ClosePixelCover;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DropPurplePixel;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PixelCoverServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PurplePixelServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Wrist_Init_AutoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Wrist_Init_AutoR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.initPlane;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorRiseyRise;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoR;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autos.Piplines.BluePipline;
import org.firstinspires.ftc.teamcode.Tuning_Variables.PoseStorage;
import org.firstinspires.ftc.teamcode.Tuning_Variables.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tuning_Variables.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Disabled
@Autonomous(group = "advanced", preselectTeleOp = "EngiNERDs_Control_RC_V2")
public class BLUE_AUTO_50Point extends LinearOpMode {
    // Calls the Variable webcam
    OpenCvWebcam webcam;
    // Calls the proper pipline in order to detect the correct color (in this case its red)
    BluePipline.bluePipline pipeline;
    // This just is determining the default position of the camera detection (this is right due to where our camera is placed)
    BluePipline.bluePipline.Detection_Positions snapshotAnalysis = BluePipline.bluePipline.Detection_Positions.RIGHT; // default



    private PIDController controller;

    // Variables for the LS motors
    public static double P = 0.021, I = 0, D = 0.0004;

    // Feedforward Component of the linear slides
    public static double f = 0;

    private double target;

    public final double ticks_in_degrees = 751.8  / 180;


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(new Pose2d());

        PurplePixelServo = hardwareMap.servo.get("PurplePixelServo");
        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");
        PixelCoverServo = hardwareMap.servo.get("PixelCoverServo");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        WristServoR = hardwareMap.servo.get("WristServoR");
        WristServoL = hardwareMap.servo.get("WristServoL");

        PixelCoverServo.setPosition(ClosePixelCover * DegreeTorque);
        PurplePixelServo.setPosition(init * DegreeTorque);
        AirplaneLaunchServo.setPosition(initPlane * DegreeTorque);


        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        PixelCoverServo.setDirection(Servo.Direction.FORWARD);
        FlooppyFloop.setDirection(Servo.Direction.FORWARD);
        WristServoL.setDirection(Servo.Direction.FORWARD);
        WristServoR.setDirection(Servo.Direction.REVERSE);
        AirplaneLaunchServo.setDirection(Servo.Direction.REVERSE);
        PurplePixelServo.setDirection(Servo.Direction.REVERSE);

        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BluePipline.bluePipline();
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

        // Let's define our trajectories
        TrajectorySequence POSITIONM = drive.trajectorySequenceBuilder(new Pose2d())

                //////////////////////////////
                // Placing the Purple Pixel //
                //////////////////////////////
                //////////////////////////////
                // Placing the Purple Pixel //
                //////////////////////////////
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    PurplePixelServo.setPosition(DropPurplePixel * DegreeTorque);
                })

                //////////////////////////////
                // Placing the Orange Pixel //
                //////////////////////////////

                .waitSeconds(4)
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    FlooppyFloop.setPosition(BackboardAutoArmsFloop * Degree5Turn);
                    FlippyFlip.setPosition(BackboardAutoArmsFlip * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-3.65, () -> {
                    WristServoR.setPosition(BackboardAutoWristBack * Degree5Turn);
                    WristServoL.setPosition(BackboardAutoWristBack * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    PixelCoverServo.setPosition(init *DegreeTorque);
                })


                //////////////////////////////
                // RESET FOR DRIVER CONTROL //
                //////////////////////////////

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    WristServoR.setPosition(init * Degree5Turn);
                    WristServoL.setPosition(init * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                    FlooppyFloop.setPosition(GroundArmsFloop * Degree5Turn);
                    FlippyFlip.setPosition(GroundArmsFlip * Degree5Turn);
                })

                .waitSeconds(50)


                .build();

        TrajectorySequence POSITIONR = drive.trajectorySequenceBuilder(new Pose2d())



                //////////////////////////////
                // Placing the Purple Pixel //
                //////////////////////////////
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    PurplePixelServo.setPosition(DropPurplePixel * DegreeTorque);
                })

                //////////////////////////////
                // Placing the Orange Pixel //
                //////////////////////////////

                .waitSeconds(4)
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    FlooppyFloop.setPosition(BackboardAutoArmsFloop * Degree5Turn);
                    FlippyFlip.setPosition(BackboardAutoArmsFlip * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-3.65, () -> {
                    WristServoR.setPosition(BackboardAutoWristBack * Degree5Turn);
                    WristServoL.setPosition(BackboardAutoWristBack * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    PixelCoverServo.setPosition(init *DegreeTorque);
                })


                //////////////////////////////
                // RESET FOR DRIVER CONTROL //
                //////////////////////////////

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    WristServoR.setPosition(init * Degree5Turn);
                    WristServoL.setPosition(init * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                    FlooppyFloop.setPosition(GroundArmsFloop * Degree5Turn);
                    FlippyFlip.setPosition(GroundArmsFlip * Degree5Turn);
                })

                .waitSeconds(50)

                .build();


        TrajectorySequence POSITIONL = drive.trajectorySequenceBuilder(new Pose2d())


                //////////////////////////////
                // Placing the Purple Pixel //
                //////////////////////////////
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    PurplePixelServo.setPosition(DropPurplePixel * DegreeTorque);
                })

                //////////////////////////////
                // Placing the Orange Pixel //
                //////////////////////////////

                .waitSeconds(4)
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    FlooppyFloop.setPosition(BackboardAutoArmsFloop * Degree5Turn);
                    FlippyFlip.setPosition(BackboardAutoArmsFlip * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-3.65, () -> {
                    WristServoR.setPosition(BackboardAutoWristBack * Degree5Turn);
                    WristServoL.setPosition(BackboardAutoWristBack * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    PixelCoverServo.setPosition(init *DegreeTorque);
                })



                //////////////////////////////
                // RESET FOR DRIVER CONTROL //
                //////////////////////////////

                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    WristServoR.setPosition(init * Degree5Turn);
                    WristServoL.setPosition(init * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                    FlooppyFloop.setPosition(GroundArmsFloop * Degree5Turn);
                    FlippyFlip.setPosition(GroundArmsFlip * Degree5Turn);
                })

                .waitSeconds(50)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes


        drive.followTrajectorySequenceAsync(POSITIONL);

        while (opModeIsActive() && !isStopRequested()) {

            // We update drive continuously in the background, regardless of state
            drive.update();

            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;
        }
    }
    public void armGround() {
        target = 0; //adjust
    }

    public void armBackBoard() {
        target = 1500; //adjust
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {

        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            controller = new PIDController(P, I,D);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            motorLiftyLift = hardwareMap.get(DcMotor.class,"motorLiftyLift");
            motorRiseyRise = hardwareMap.get(DcMotor.class,"motorRiseyRise");

            motorLiftyLift.setDirection(DcMotorSimple.Direction.REVERSE);


            target = 0;
        }


        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift

            controller.setPID(P, I,D);
            int LinearSlide_Pos1 = motorRiseyRise.getCurrentPosition();
            int LinearSlide_Pos2 = motorLiftyLift.getCurrentPosition();

            double pid = controller.calculate(LinearSlide_Pos1,target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double power = pid + ff;

            motorRiseyRise.setPower(power);
            motorLiftyLift.setPower(power);

            telemetry.addData("Risey Rise Pos", LinearSlide_Pos1);
            telemetry.addData("LiftyLift Pos", LinearSlide_Pos2);
            telemetry.addData("Target Pos", target);
            telemetry.update();

        }
    }
}
