package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMountServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Close;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeAirplane;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeArm;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeWrist;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.InitArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.InitArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.OpenAuto;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack2ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack2ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack4ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack4ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack5ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack5ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateGround;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateInit;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorRiseyRise;

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
import org.firstinspires.ftc.teamcode.Examples.BluePipline;
import org.firstinspires.ftc.teamcode.Examples.RedPipline;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

        LeftClaw = hardwareMap.servo.get("LeftClaw");
        RightClaw = hardwareMap.servo.get("RightClaw");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        GearServo = hardwareMap.servo.get("GearServo");
        AirplaneMountServo = hardwareMap.servo.get("AirplaneMountServo");
        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");

        FlooppyFloop.setDirection(Servo.Direction.FORWARD);
        LeftClaw.setDirection(Servo.Direction.REVERSE);


        AirplaneMountServo.setPosition(0 * DegreeAirplane);
        LeftClaw.setPosition(0 * DegreeClaw); // Closes
        RightClaw.setPosition(0 * DegreeClaw); // Closes
        FlooppyFloop.setPosition(InitArmRotateFloop * DegreeArm); // Rotates at an angle forwards
        FlippyFlip.setPosition(InitArmRotateFlip * DegreeArm); // rotates at an angle forwards
        GearServo.setPosition(WristRotateInit * DegreeWrist); // Rotates into the air


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
                .waitSeconds(.5)
                .UNSTABLE_addDisplacementMarkerOffset(-0.5, () -> {
                    FlooppyFloop.setPosition(Stack2ArmRotateFloop * DegreeArm);
                    FlippyFlip.setPosition(Stack2ArmRotateFlip * DegreeArm);
                })
                .UNSTABLE_addDisplacementMarkerOffset(-0.15, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })
                .splineTo(new Vector2d(23,8),Math.toRadians(90))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-.45, () -> {
                    LeftClaw.setPosition(Open * DegreeClaw);
                })
                .lineToLinearHeading(new Pose2d(23,-2,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(40,-2,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(40,23,Math.toRadians(94)))
                .lineToLinearHeading(new Pose2d(40,58,Math.toRadians(94)))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    armBackBoard();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    GearServo.setPosition(80 * DegreeWrist);
                })
                .lineToLinearHeading(new Pose2d(9,58,Math.toRadians(94)))
                .lineToLinearHeading(new Pose2d(9,69,Math.toRadians(95)))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    RightClaw.setPosition(Open * DegreeClaw);
                })
                .lineToLinearHeading(new Pose2d(11,45,Math.toRadians(95)))
                .lineToLinearHeading(new Pose2d(33,45,Math.toRadians(-95)))
                .lineToLinearHeading(new Pose2d(33,75,Math.toRadians(-95)))
                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-5, () -> {
                    LeftClaw.setPosition(Close * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(-5, () -> {
                    RightClaw.setPosition(Close * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    FlooppyFloop.setPosition(Stack2ArmRotateFloop * DegreeArm);
                    FlippyFlip.setPosition(Stack2ArmRotateFlip * DegreeArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    armGround();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })
                .waitSeconds(25)
                .build();

        TrajectorySequence POSITIONM = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(.5)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    FlooppyFloop.setPosition(Stack2ArmRotateFloop * DegreeArm);
                    FlippyFlip.setPosition(Stack2ArmRotateFlip * DegreeArm);
                })
                .UNSTABLE_addDisplacementMarkerOffset(0.15, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })
                .lineToLinearHeading(new Pose2d(42,8,Math.toRadians(180)))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-.45, () -> {
                    LeftClaw.setPosition(Open * DegreeClaw);
                })
                .lineToLinearHeading(new Pose2d(50,18,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(44,26,Math.toRadians(94)))
                .lineToLinearHeading(new Pose2d(44,69,Math.toRadians(96)))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    armBackBoard();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    GearServo.setPosition(90 * DegreeWrist);
                })
                .lineToLinearHeading(new Pose2d(19,69,Math.toRadians(96)))
                .lineToLinearHeading(new Pose2d(19,78.5,Math.toRadians(91)))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.25, () -> {
                    RightClaw.setPosition(Open * DegreeClaw);
                })
                .lineToLinearHeading(new Pose2d(19,68,Math.toRadians(91)))
                .lineToLinearHeading(new Pose2d(45,68,Math.toRadians(-91)))
                .lineToLinearHeading(new Pose2d(45,88,Math.toRadians(-91)))
                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-5, () -> {
                    LeftClaw.setPosition(Close * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(-5, () -> {
                    RightClaw.setPosition(Close * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    FlooppyFloop.setPosition(Stack2ArmRotateFloop * DegreeArm);
                    FlippyFlip.setPosition(Stack2ArmRotateFlip * DegreeArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    armGround();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })

                .waitSeconds(25)
                .build();

        TrajectorySequence POSITIONR = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(.5)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    FlooppyFloop.setPosition(Stack2ArmRotateFloop * DegreeArm);
                    FlippyFlip.setPosition(Stack2ArmRotateFlip * DegreeArm);
                })
                .UNSTABLE_addDisplacementMarkerOffset(0.15, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })
                .lineToLinearHeading(new Pose2d(30,0,Math.toRadians(-90)))
                .waitSeconds(.5)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(-.45, () -> {
                    LeftClaw.setPosition(OpenAuto * DegreeClaw);
                })
                .lineToLinearHeading(new Pose2d(46,0,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(47,0,Math.toRadians(93)))
                .lineToLinearHeading(new Pose2d(47,60,Math.toRadians(93)))
                .lineToLinearHeading(new Pose2d(33,60,Math.toRadians(93)))



                .waitSeconds(25)

                .build();

        waitForStart();

        if (isStopRequested()) return;

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
