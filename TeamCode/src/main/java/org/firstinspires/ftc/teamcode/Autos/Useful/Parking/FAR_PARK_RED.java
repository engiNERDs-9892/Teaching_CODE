package org.firstinspires.ftc.teamcode.Autos.Useful.Parking;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AutoWristGround;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ClosePixelCover;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ParkAutoArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ParkAutoArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PixelCoverServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PurplePixelServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.initPlane;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autos.Useful.Piplines.BluePipline;
import org.firstinspires.ftc.teamcode.Tuning_Variables.PoseStorage;
import org.firstinspires.ftc.teamcode.Tuning_Variables.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tuning_Variables.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


//@Disabled
@Autonomous(group = "advanced", preselectTeleOp = "EngiNERDs_Control_RC_V2_RED")
public class FAR_PARK_RED extends LinearOpMode {

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


        TrajectorySequence PARK = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-50, -2, Math.toRadians(-79)))
                .lineToLinearHeading(new Pose2d(-56, 70, Math.toRadians(-79)))
                .waitSeconds(4)

                /*
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    FlooppyFloop.setPosition(ParkAutoArmsFloop * Degree5Turn);
                    FlippyFlip.setPosition(ParkAutoArmsFlip * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-4, () -> {
                    WristServoR.setPosition(AutoWristGround * Degree5Turn);
                    WristServoL.setPosition(AutoWristGround * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    PixelCoverServo.setPosition(init * DegreeTorque);
                })
                .waitSeconds(10)
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(-8, () -> {
                    WristServoR.setPosition(init * Degree5Turn);
                    WristServoL.setPosition(init * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-7, () -> {
                    FlooppyFloop.setPosition(GroundArmsFloop * Degree5Turn);
                    FlippyFlip.setPosition(GroundArmsFlip * Degree5Turn);
                })
                */
                .build();


        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes


        drive.followTrajectorySequenceAsync(PARK);

        while (opModeIsActive() && !isStopRequested()) {

            // We update drive continuously in the background, regardless of state
            drive.update();


            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;
        }
    }
}
