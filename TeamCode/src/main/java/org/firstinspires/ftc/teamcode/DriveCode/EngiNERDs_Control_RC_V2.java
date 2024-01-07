package org.firstinspires.ftc.teamcode.DriveCode;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMountServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Close;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GroundArmRotateR;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorRiseyRise;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.slideySlideMax;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.slideySlideMin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
//@Disabled
public class EngiNERDs_Control_RC_V2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        new EngiNERDs_Variables(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {

            if (gamepad1.right_trigger != 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y,
                                gamepad1.left_stick_x,
                                gamepad1.right_stick_x
                        )
                );
            }

            if (gamepad1.left_trigger != 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y * .2,
                                gamepad1.left_stick_x * .2,
                                gamepad1.right_stick_x * .2
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y * .7,
                                gamepad1.left_stick_x * .7,
                                gamepad1.right_stick_x * .7
                        )
                );
            }
        }
    }
}
