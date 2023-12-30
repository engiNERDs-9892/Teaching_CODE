package org.firstinspires.ftc.teamcode.DriveCode;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Close;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;
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
@Disabled
public class EngiNERDs_Control_RC_V2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        new EngiNERDs_Variables(hardwareMap);

        boolean Right_Claw_Toggle = false;

        boolean Left_Claw_Toggle = false;

        boolean Arm_Toggle = false;

        boolean Hook_Toggle = false;

        waitForStart();

        while (!isStopRequested()) {
            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Encoder tracker for the LS so it doesn't go over the limit / to high or low
            int LiftyLiftPos = motorLiftyLift.getCurrentPosition();
            int RiseyRisePos = motorRiseyRise.getCurrentPosition();

            // Joystick Values for the Linear slides
            double RaiseandLower = -gamepad2.right_stick_y;


            if (gamepad1.right_trigger != 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }

            if (gamepad1.left_trigger != 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * .2,
                                -gamepad1.left_stick_x * .2,
                                -gamepad1.right_stick_x * .2
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * .7,
                                -gamepad1.left_stick_x * .7,
                                -gamepad1.right_stick_x * .7
                        )
                );
            }


            if (LiftyLiftPos >= slideySlideMin && RiseyRisePos >= slideySlideMin
                    && LiftyLiftPos <= slideySlideMax && RiseyRisePos <= slideySlideMax) {

                // If you are trying to raise the linear Slide
                // Then raise the linear slides!
                if (RaiseandLower < -0.05) {
                    motorRiseyRise.setPower(RaiseandLower);
                    motorLiftyLift.setPower(RaiseandLower);
                }


                // if you are trying to lower the linear Slide
                // Then lower the linear slides!
                if (RaiseandLower > 0.05) {
                    motorRiseyRise.setPower(RaiseandLower);
                    motorLiftyLift.setPower(RaiseandLower);
                }

                // If you are not pushing on the joystick the power = 0
                // This is mainly to prevent stick drift
                if ((RaiseandLower >= -0.05) && (RaiseandLower <= 0.05)) {
                    motorRiseyRise.setPower(0);
                    motorLiftyLift.setPower(0);
                }
            }

            if (LiftyLiftPos < slideySlideMin || RiseyRisePos < slideySlideMin) {

                if (RaiseandLower > 0.05) {
                    motorRiseyRise.setPower(RaiseandLower);
                    motorLiftyLift.setPower(RaiseandLower);
                } else {
                    motorRiseyRise.setPower(0);
                    motorLiftyLift.setPower(0);
                }

            }

            if (LiftyLiftPos > slideySlideMax || RiseyRisePos > slideySlideMax) {
                if (RaiseandLower < -0.05) {
                    motorRiseyRise.setPower(RaiseandLower);
                    motorLiftyLift.setPower(RaiseandLower);
                } else {
                    motorRiseyRise.setPower(0);
                    motorLiftyLift.setPower(0);
                }
            }


            // Toggle / Raise and Lower for the Arms
            if (currentGamepad2.a && !previousGamepad2.a) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                Arm_Toggle = !Arm_Toggle;
            }
            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (Arm_Toggle) {
                FlooppyFloop.setPosition(.5);
                FlippyFlip.setPosition(.5);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                FlooppyFloop.setPosition(.02);
                FlippyFlip.setPosition(.98);
            }


            // Claws

            // Toggle / Close & Open for the Right claw
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                Right_Claw_Toggle = !Right_Claw_Toggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (Right_Claw_Toggle) {
                RightClaw.setPosition(.77);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                RightClaw.setPosition(1);
            }


            // Toggle / Close & Open for the Left claw
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                Left_Claw_Toggle = !Left_Claw_Toggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (Left_Claw_Toggle) {
                LeftClaw.setPosition(Open);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                LeftClaw.setPosition(Close);
            }


            if (Math.abs(gamepad2.left_stick_y) >= 0.5) {
                GearServo.setPosition((GearServo.getPosition() + 0.005 * Math.signum(gamepad2.left_stick_y)));
            }


            if (gamepad1.a) {
                AirplaneServo.setPosition(0.7);
            }
        }
    }
}
