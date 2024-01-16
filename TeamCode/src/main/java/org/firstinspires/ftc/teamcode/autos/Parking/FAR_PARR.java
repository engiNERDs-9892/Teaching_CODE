package org.firstinspires.ftc.teamcode.autos.Parking;

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
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack2ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack2ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack5ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack5ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateGround;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", preselectTeleOp = "EngiNERDs_Control_RC_V2")
//@Disabled
public class FAR_PARR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        LeftClaw = hardwareMap.servo.get("LeftClaw");
        RightClaw = hardwareMap.servo.get("RightClaw");
        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        GearServo = hardwareMap.servo.get("GearServo");
        AirplaneMountServo = hardwareMap.servo.get("AirplaneMountServo");
        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");

        LeftClaw.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);

        AirplaneMountServo.setPosition(0 * DegreeAirplane);
        LeftClaw.setPosition(0 * DegreeClaw); // Closes
        RightClaw.setPosition(0 * DegreeClaw); // Closes

        // this call sets the servos during initialization
        FlooppyFloop.setPosition(45 * DegreeArm); // Rotates at an angle forwards
        FlippyFlip.setPosition(50 * DegreeArm); // rotates at an angle forwards
        GearServo.setPosition(225 * DegreeWrist); // Rotates into the air
        TrajectorySequence tajpark = drive.trajectorySequenceBuilder(new Pose2d())


                .lineToLinearHeading(new Pose2d(54,0,Math.toRadians(-90)))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    FlooppyFloop.setPosition(Stack5ArmRotateFloop * DegreeArm);
                    FlippyFlip.setPosition(Stack5ArmRotateFlip * DegreeArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })
                .forward(80)
                .waitSeconds(2)


                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    LeftClaw.setPosition(Open * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    RightClaw.setPosition(Open * DegreeClaw);
                })
                .back(5)

                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    LeftClaw.setPosition(Close * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    RightClaw.setPosition(Close * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    FlooppyFloop.setPosition(Stack2ArmRotateFloop * DegreeArm);
                    FlippyFlip.setPosition(Stack2ArmRotateFlip * DegreeArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })


                .build();


        TrajectorySequence trajL = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(.5)
                .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                    FlooppyFloop.setPosition(Stack2ArmRotateFloop * DegreeArm);
                    FlippyFlip.setPosition(Stack2ArmRotateFlip * DegreeArm);
                })
                .UNSTABLE_addDisplacementMarkerOffset(0.15, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })
                .splineTo(new Vector2d(29,6),Math.toRadians(90))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-.45, () -> {
                    LeftClaw.setPosition(Open * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    FlooppyFloop.setPosition(55 * DegreeArm);
                    FlippyFlip.setPosition(60 * DegreeArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
                    GearServo.setPosition(75 * DegreeWrist);
                })
                .lineToLinearHeading(new Pose2d(28,-49.5,Math.toRadians(-85)))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(-.45, () -> {
                    RightClaw.setPosition(Open * DegreeClaw);
                })

                .lineToLinearHeading(new Pose2d(25,-36,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(0,-36,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-6,-65,Math.toRadians(90)))

                .waitSeconds(5)
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    LeftClaw.setPosition(Close * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    RightClaw.setPosition(Close * DegreeClaw);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    FlooppyFloop.setPosition(Stack2ArmRotateFloop * DegreeArm);
                    FlippyFlip.setPosition(Stack2ArmRotateFlip * DegreeArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    GearServo.setPosition(WristRotateGround * DegreeWrist);
                })
                .build();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySequence(tajpark);

        sleep(20000);
    }
}
