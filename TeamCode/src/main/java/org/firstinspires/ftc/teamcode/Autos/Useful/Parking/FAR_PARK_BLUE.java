package org.firstinspires.ftc.teamcode.Autos.Useful.Parking;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AutoWristGround;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ClosePixelCover;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ParkAutoArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ParkAutoArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.initPlane;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tuning_Variables.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tuning_Variables.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
//@Disabled
@Autonomous(group = "drive")
public class FAR_PARK_BLUE extends LinearOpMode {

    public Servo PurplePixelServo;
    public Servo AirplaneLaunchServo;
    public Servo PixelCoverServo;
    public Servo FlooppyFloop;
    public Servo FlippyFlip;
    public Servo WristServoR;
    public Servo WristServoL;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        TrajectorySequence Park = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-50, 2, Math.toRadians(79)))
                .lineToLinearHeading(new Pose2d(-56, -70, Math.toRadians(79)))
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

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequenceAsync(Park);
        }
    }
}