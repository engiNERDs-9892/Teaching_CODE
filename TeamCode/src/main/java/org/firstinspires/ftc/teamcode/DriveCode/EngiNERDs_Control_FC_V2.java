package org.firstinspires.ftc.teamcode.DriveCode;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMountServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMount_Rotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.BackboardArmRotat;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.BackboardArmRotate;
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
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RESETARM;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RESET_ARM;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.ShootPlane;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack1ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack1ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack2ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack2ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack3ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack3ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack4ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack4ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack5ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack5ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateBackboard;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateGround;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorRiseyRise;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class EngiNERDs_Control_FC_V2 extends LinearOpMode {

    public enum LiftState {
        LIFT_START,
        ARM_ROTATED,
        GEARSERVO_ROTATED,
        OPEN_CLAWS,
        CLOSED_LEFT,
        CLOSED_RIGHT,
        LS_LIFTED,
        LS_LOWERED,
        WRIST_RESET,
        FULL_RESET,
        IDLE
    }

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_START;

    @Override
    public void runOpMode() throws InterruptedException {

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        boolean Right_Claw_Toggle = false;

        boolean Left_Claw_Toggle = false;

        boolean AirplaneMount_Toggle = false;


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        new EngiNERDs_Variables(hardwareMap);

        waitForStart();

        FlooppyFloop.setPosition(1795 * DegreeArm);
        FlippyFlip.setPosition(13 * DegreeArm);
        GearServo.setPosition(WristRotateGround * DegreeWrist);

        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            double RaiseandLower = -gamepad2.left_stick_y;

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            Vector2d input = new Vector2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x
            ).rotated(poseEstimate.getHeading());

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately

           if (gamepad1.right_trigger != 0) {
               drive.setWeightedDrivePower(
                       new Pose2d(
                               input.getX(),
                               input.getY(),
                               gamepad1.right_stick_x
                       )
               );
           }

            if (gamepad1.left_trigger != 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX() *.2 ,
                                input.getY() * .2,
                                gamepad1.right_stick_x * .2
                        )
                );
            }

            else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX() *.6 ,
                                input.getY() * .6,
                                gamepad1.right_stick_x * .6
                        )
                );
            }


            if (RaiseandLower == 0) {
                motorRiseyRise.setPower(0);
                motorLiftyLift.setPower(0);
            } else {
                // move slide up for RaiseandLower < 0, move slide down on RaiseandLower > 0
                motorRiseyRise.setPower(RaiseandLower * 1);
                motorLiftyLift.setPower(RaiseandLower * 1);
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
                RightClaw.setPosition(Open * DegreeClaw);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                RightClaw.setPosition(Close * DegreeClaw);
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
                LeftClaw.setPosition(Open * DegreeClaw);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                LeftClaw.setPosition(Close * DegreeClaw);
            }


            if (Math.abs(gamepad2.right_stick_y) >= 0.5) {
                GearServo.setPosition((GearServo.getPosition() + 0.0005 * Math.signum(-gamepad2.right_stick_y)));
            }


            if (gamepad1.right_bumper) {
                AirplaneLaunchServo.setPosition(ShootPlane * DegreeAirplane);
            }

            // Toggle / Close & Open for the Left claw
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                AirplaneMount_Toggle = !AirplaneMount_Toggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (AirplaneMount_Toggle) {
                AirplaneMountServo.setPosition(AirplaneMount_Rotate * DegreeAirplane);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                AirplaneMountServo.setPosition(Close * DegreeAirplane);
            }


            ////////////////////////////////////////////////
            // Macro - Stack 5 High / Grab 2 from Stack 5 //
            ////////////////////////////////////////////////
            switch (liftState) {
                case LIFT_START:
                    // Waiting for some input
                    if (gamepad1.dpad_up) {
                        // x is pressed, start extending
                        FlooppyFloop.setPosition(Stack5ArmRotateFloop * DegreeArm);
                        FlippyFlip.setPosition(Stack5ArmRotateFlip * DegreeArm);
                        liftState = LiftState.ARM_ROTATED;
                    }
                    break;

                case ARM_ROTATED:
                    if ((FlooppyFloop.getPosition() == (Stack5ArmRotateFloop * DegreeArm)) || (FlippyFlip.getPosition() == (Stack5ArmRotateFlip * DegreeArm))) {

                        GearServo.setPosition(WristRotateGround * DegreeWrist);
                        liftState = LiftState.LIFT_START;
                    }
                    break;
            }


            ////////////////////////////////////////////////
            // Macro - Stack 4 High / Grab 2 from Stack 4 //
            ////////////////////////////////////////////////
            switch (liftState) {
                case LIFT_START:
                    // Waiting for some input
                    if (gamepad1.dpad_left) {
                        // x is pressed, start extending
                        FlooppyFloop.setPosition(Stack4ArmRotateFloop * DegreeArm);
                        FlippyFlip.setPosition(Stack4ArmRotateFlip * DegreeArm);
                        liftState = LiftState.ARM_ROTATED;
                    }
                    break;

                case ARM_ROTATED:
                    if ((FlooppyFloop.getPosition() == (Stack4ArmRotateFloop * DegreeArm)) || (FlippyFlip.getPosition() == (Stack4ArmRotateFlip * DegreeArm))) {

                        GearServo.setPosition(WristRotateGround * DegreeWrist);
                        liftState = LiftState.LIFT_START;
                    }
                    break;
            }

            ////////////////////////////////////////////////
            // Macro - Stack 2 High / Grab 2 from Stack 2 //
            ////////////////////////////////////////////////
            switch (liftState) {
                case LIFT_START:
                    // Waiting for some input
                    if (gamepad1.dpad_right) {
                        // x is pressed, start extending
                        FlooppyFloop.setPosition(Stack3ArmRotateFloop * DegreeArm);
                        FlippyFlip.setPosition(Stack3ArmRotateFlip * DegreeArm);
                        liftState = LiftState.ARM_ROTATED;
                    }
                    break;

                case ARM_ROTATED:
                    if ((FlooppyFloop.getPosition() == (Stack3ArmRotateFloop * DegreeArm)) || (FlippyFlip.getPosition() == (Stack3ArmRotateFlip * DegreeArm))) {

                        GearServo.setPosition(WristRotateGround * DegreeWrist);
                        liftState = LiftState.LIFT_START;
                    }
                    break;
            }

            ////////////////////////////////////////////////
            // Macro - Stack 2 High / Grab 2 from Stack 2 //
            ////////////////////////////////////////////////
            switch (liftState) {
                case LIFT_START:
                    // Waiting for some input
                    if (gamepad1.dpad_down) {
                        // x is pressed, start extending
                        FlooppyFloop.setPosition(Stack2ArmRotateFloop * DegreeArm);
                        FlippyFlip.setPosition(Stack2ArmRotateFlip * DegreeArm);
                        liftState = LiftState.ARM_ROTATED;
                    }
                    break;

                case ARM_ROTATED:
                    if ((FlooppyFloop.getPosition() == (Stack2ArmRotateFloop * DegreeArm)) || (FlippyFlip.getPosition() == (Stack2ArmRotateFlip * DegreeArm))) {

                        GearServo.setPosition(WristRotateGround * DegreeWrist);
                        liftState = LiftState.LIFT_START;
                    }
                    break;
            }

            ////////////////////////////////////////////////
            // Macro - HANG //
            ////////////////////////////////////////////////
            switch (liftState) {
                case LIFT_START:
                    // Waiting for some input
                    if (gamepad2.x) {
                        // x is pressed, start extending
                        FlooppyFloop.setPosition(BackboardArmRotate * DegreeArm);
                        FlippyFlip.setPosition(BackboardArmRotat * DegreeArm);
                        liftState = LiftState.ARM_ROTATED;
                    }
                    break;

                case ARM_ROTATED:
                    if ((FlooppyFloop.getPosition() == (BackboardArmRotate * DegreeArm)) || (FlippyFlip.getPosition() == (BackboardArmRotat * DegreeArm))) {

                        GearServo.setPosition(WristRotateBackboard * DegreeWrist);
                        liftState = LiftState.LIFT_START;
                    }
                    break;
            }




            switch (liftState) {
                case LIFT_START:
                    // Waiting for some input
                    if (gamepad1.triangle || gamepad1.y || gamepad2.y) {
                        // x is pressed, start extending
                        FlooppyFloop.setPosition(1795 * DegreeArm);
                        FlippyFlip.setPosition(13 * DegreeArm);
                        liftState = LiftState.ARM_ROTATED;
                    }

                case ARM_ROTATED:
                   if ((FlooppyFloop.getPosition() == (Stack1ArmRotateFloop * DegreeArm)) || FlippyFlip.getPosition() == (Stack1ArmRotateFlip * DegreeArm))
                    GearServo.setPosition(3 * DegreeWrist);
                    liftState = LiftState.LIFT_START;
                       break;


                default:
                    // should never be reached, as liftState should never be null
                    liftState = LiftState.LIFT_START;

            }


            if ((gamepad1.back || gamepad2.back || gamepad1.options) && liftState != LiftState.LIFT_START) {
                liftState = LiftState.LIFT_START;
            }


            // Telemetry
            telemetry.addData("LEFT LS POS", motorLiftyLift.getCurrentPosition());
            telemetry.addData("RIGHT LS POS", motorRiseyRise.getCurrentPosition());
            telemetry.addData("GEAR SERVO POS", GearServo.getPosition());
            telemetry.addData("LEFT ARM POS", FlooppyFloop.getPosition());
            telemetry.addData("RIGHT ARM POS", FlippyFlip.getPosition());
            telemetry.addData("LEFT CLAW POS", LeftClaw.getPosition());
            telemetry.addData("RIGHT CLAW POS", RightClaw.getPosition());
            telemetry.update();
        }
    }
}

