package org.firstinspires.ftc.teamcode.Autos.Auto50Point;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.InitArms;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.InitWrist;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorRiseyRise;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@Autonomous(group = "advanced", preselectTeleOp = "RESET_HARDWARE")
public class BLUE_AUTO_50Point_Far extends LinearOpMode {
    // Calls the Variable webcam
    OpenCvWebcam webcam;
    // Calls the proper pipline in order to detect the correct color (in this case its red)
    BluePipline.bluePipline pipeline;
    // This just is determining the default position of the camera detection (this is right due to where our camera is placed)
    BluePipline.bluePipline.Detection_Positions snapshotAnalysis = BluePipline.bluePipline.Detection_Positions.RIGHT; // default



    private PIDController controller;
    private PIDController controller2;

    // Variables For the left side calculations
    public static double Pl = 0.021, Il = 0, Dl = 0.0004;

    // Variables For the right side calculations
    public static double Pr = 0.021, Ir = 0, Dr = 0.0004;

    // Feedforward Component of the linear slides
    public static double f = 0;

    private double target;

    public final double ticks_in_degrees = 1993.6 / 180;


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(new Pose2d());

        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        WristServoL = hardwareMap.servo.get("WristServoL");
        WristServoR = hardwareMap.servo.get("WristServoL");
        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");

        FlooppyFloop.setPosition(InitArms * Degree5Turn); // Rotates at an angle forwards
        FlippyFlip.setPosition(InitArms * Degree5Turn); // rotates at an angle forwards
        WristServoL.setPosition(InitWrist * Degree5Turn); // Rotates into the air
        WristServoR.setPosition(InitWrist * Degree5Turn);


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
            telemetry.addData("Floop", FlooppyFloop.getPosition());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();


        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);

        telemetry.update();





        // Let's define our trajectories
        TrajectorySequence POSITIONL = drive.trajectorySequenceBuilder(new Pose2d())

                .splineTo(new Vector2d(25,8),Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(23,-2,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(40,-2,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(40,23,Math.toRadians(94)))
                .lineToLinearHeading(new Pose2d(40,58,Math.toRadians(94)))
                .lineToLinearHeading(new Pose2d(10,58,Math.toRadians(94)))
                .lineToLinearHeading(new Pose2d(10,71,Math.toRadians(95)))
                .lineToLinearHeading(new Pose2d(10,60,Math.toRadians(95)))
                .lineToLinearHeading(new Pose2d(36,60,Math.toRadians(-95)))
                .lineToLinearHeading(new Pose2d(36,75,Math.toRadians(-95)))
                .build();

        TrajectorySequence POSITIONM = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(39,6,Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(50,10,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(40,26,Math.toRadians(94)))
                .lineToLinearHeading(new Pose2d(40,56,Math.toRadians(96)))
                .lineToLinearHeading(new Pose2d(19,67,Math.toRadians(96)))
                .lineToLinearHeading(new Pose2d(19,75,Math.toRadians(91)))
                .lineToLinearHeading(new Pose2d(24,65,Math.toRadians(91)))
                .lineToLinearHeading(new Pose2d(35,65,Math.toRadians(-91)))
                .lineToLinearHeading(new Pose2d(35,83,Math.toRadians(-91)))
                .build();

        TrajectorySequence POSITIONR = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30,0,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(46,0,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(47,0,Math.toRadians(93)))
                .lineToLinearHeading(new Pose2d(47,60,Math.toRadians(93)))
                .lineToLinearHeading(new Pose2d(33,60,Math.toRadians(93)))



                .waitSeconds(25)

                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequenceAsync(POSITIONM);

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
        target = 0; //adjust TODO
    }

    public void armBackBoard() {
        target = 2700; //adjust TODO
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {

        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            controller = new PIDController(Pr, Ir,Dr);
            controller2 = new PIDController(Pl, Il,Dl);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            motorLiftyLift = hardwareMap.get(DcMotor.class,"motorLiftyLift");
            motorRiseyRise = hardwareMap.get(DcMotor.class,"motorRiseyRise");

            target = 0;
        }


        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift

            controller.setPID(Pr, Ir,Dr);
            controller2.setPID(Pl, Il,Dl);
            int LinearSlide_Pos1 = motorRiseyRise.getCurrentPosition();
            int LinearSlide_Pos2 = motorLiftyLift.getCurrentPosition();

            double pidR = controller.calculate(LinearSlide_Pos1,target);
            double pidL = controller2.calculate(LinearSlide_Pos2, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double powerR = pidR + ff;
            double powerL = pidL + ff;

            motorRiseyRise.setPower(powerR);
            motorLiftyLift.setPower(powerL);

            telemetry.addData("Risey Rise Pos", LinearSlide_Pos1);
            telemetry.addData("LiftyLift Pos", LinearSlide_Pos2);
            telemetry.addData("Target Pos", target);
            telemetry.update();

        }

    }
}
