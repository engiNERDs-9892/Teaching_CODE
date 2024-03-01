package org.firstinspires.ftc.teamcode.Autos.Auto50Point;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServo;
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
import org.firstinspires.ftc.teamcode.Autos.Piplines.RedPipline;
import org.firstinspires.ftc.teamcode.Tuning_Variables.PoseStorage;
import org.firstinspires.ftc.teamcode.Tuning_Variables.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tuning_Variables.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(group = "advanced", preselectTeleOp = "EngiNERDs_Control_RC_V2")
public class RED_AUTO_50Point extends LinearOpMode {
    // Calls the Variable webcam
    OpenCvWebcam webcam;
    // Calls the proper pipline in order to detect the correct color (in this case its red)
    RedPipline.redPipline pipeline;
    // This just is determining the default position of the camera detection (this is right due to where our camera is placed)
    RedPipline.redPipline.Detection_Positions snapshotAnalysis = RedPipline.redPipline.Detection_Positions.RIGHT; // default



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
        WristServo = hardwareMap.servo.get("WristServo");
        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");

        FlooppyFloop.setDirection(Servo.Direction.FORWARD);


        FlooppyFloop.setPosition(0 * DegreeTorque); // Rotates at an angle forwards
        FlippyFlip.setPosition(0 * DegreeTorque); // rotates at an angle forwards
        WristServo.setPosition(0 * DegreeTorque); // Rotates into the air


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





        // Let's define our trajectories
        TrajectorySequence POSITIONL = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(26,6),Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(29,-20,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(29,-21,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(29,-28,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(27,-15,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(2,-15,Math.toRadians(83)))
                .lineToLinearHeading(new Pose2d(-2,-39,Math.toRadians(83)))
                .build();

        TrajectorySequence POSITIONM = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(22,0,Math.toRadians(2)))
                .lineToLinearHeading(new Pose2d(22,-15,Math.toRadians(2)))
                .lineToLinearHeading(new Pose2d(23,-15,Math.toRadians(-87)))
                .lineToLinearHeading(new Pose2d(23,-33,Math.toRadians(-87)))
                .lineToLinearHeading(new Pose2d(-1,-36,Math.toRadians(87)))
                .lineToLinearHeading(new Pose2d(-1,-47,Math.toRadians(87)))
                .build();

        TrajectorySequence POSITIONR = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(29,-10.5,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(22,-16,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(22,-17,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(22,-28,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(22,-25,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(7,-26,Math.toRadians(85)))
                .lineToLinearHeading(new Pose2d(7,-45,Math.toRadians(85)))
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
        target = 0; //adjust TODO
    }

    public void armBackBoard() {
        target = 1500; //adjust TODO
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
