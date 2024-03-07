package org.firstinspires.ftc.teamcode.Autos.Parking;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AutoWristGround;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ClosePixelCover;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ParkAutoArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ParkAutoArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Wrist_Init_AutoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Wrist_Init_AutoR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.initFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.initPlane;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tuning_Variables.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tuning_Variables.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", preselectTeleOp = "EngiNERDs_Control_RC_V2")
@Disabled
public class SHORT_PARKING extends LinearOpMode {

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

        FlippyFlip.setDirection(Servo.Direction.FORWARD);
        PixelCoverServo.setDirection(Servo.Direction.FORWARD);
        FlooppyFloop.setDirection(Servo.Direction.FORWARD);
        WristServoL.setDirection(Servo.Direction.FORWARD);
        WristServoR.setDirection(Servo.Direction.REVERSE);
        AirplaneLaunchServo.setDirection(Servo.Direction.REVERSE);
        PurplePixelServo.setDirection(Servo.Direction.REVERSE);



        TrajectorySequence tajpark = drive.trajectorySequenceBuilder(new Pose2d())

                .back(40)
                .waitSeconds(4)
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
                .build();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySequence(tajpark);

        sleep(20000);
    }
}
