package org.firstinspires.ftc.teamcode.Testing.Auto;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.BackboardAutoArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.BackboardAutoArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.BackboardAutoWristBack;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ClosePixelCover;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PixelCoverServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PurplePixelServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoR;
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
//@Disabled
public class OrangePixelAction extends LinearOpMode {

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


        TrajectorySequence OrangePixel = drive.trajectorySequenceBuilder(new Pose2d())

                .back(2)
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    FlooppyFloop.setPosition(BackboardAutoArmsFloop * Degree5Turn);
                    FlippyFlip.setPosition(BackboardAutoArmsFlip * Degree5Turn);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.65, () -> {
                    WristServoR.setPosition(BackboardAutoWristBack * Degree5Turn);
                    WristServoL.setPosition(BackboardAutoWristBack * Degree5Turn);
                })
                .back(2)
                .waitSeconds(20)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    PixelCoverServo.setPosition(init *DegreeTorque);
                })
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


        drive.followTrajectorySequence(OrangePixel);

        sleep(20000);
    }
}
