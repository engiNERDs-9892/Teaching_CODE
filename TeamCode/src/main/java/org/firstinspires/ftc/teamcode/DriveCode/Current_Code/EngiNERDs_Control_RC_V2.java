package org.firstinspires.ftc.teamcode.DriveCode.Current_Code;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.BackboardDriverArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.BackboardDriverArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ClosePixelCover;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FirstSetlineBackboardDriverArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FirstSetlineBackboardDriverArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.GroundArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.LaunchPlane;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PixelCoverServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.SlightlyAboveGroundDriverArmsFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.SlightlyAboveGroundDriverArmsFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.initFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorINTAKE;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorRiseyRise;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables;
import org.firstinspires.ftc.teamcode.Tuning_Variables.SampleMecanumDrive;

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

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        new EngiNERDs_Variables(hardwareMap);

        boolean PixelCover_Toggle = false;

        waitForStart();

        AirplaneLaunchServo.setPosition(init * DegreeTorque);
        FlooppyFloop.setPosition(init * Degree5Turn);
        FlippyFlip.setPosition(initFlip * Degree5Turn);
        WristServoL.setPosition(init * Degree5Turn);
        WristServoR.setPosition(init * Degree5Turn);
        while (opModeIsActive()) {



            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            double RaiseandLower = -gamepad2.left_stick_y;

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


        ////////////////////////////////////////////////////
        // Movement Code ///////////////////////////////////
        ////////////////////////////////////////////////////


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

            ////////////////////////////////////////////////////////////////////////
            // Arm Playing Mechanism ///////////////////////////////////////////////                                             /
            ////////////////////////////////////////////////////////////////////////


            // Linear Slide Code (Up / Down is in the else statement)
            if (RaiseandLower == 0) {
                motorRiseyRise.setPower(0);
                motorLiftyLift.setPower(0);
            }

            else {
                // move slide up for RaiseandLower < 0, move slide down on RaiseandLower > 0
                motorRiseyRise.setPower(RaiseandLower * 1);
                motorLiftyLift.setPower(RaiseandLower * 1);
            }

            // Wrist Joint Servos
            if (Math.abs(gamepad2.right_stick_y) >= 0.5) {
                WristServoL.setPosition((WristServoL.getPosition() + 0.001 * Math.signum(-gamepad2.right_stick_y)));
                WristServoR.setPosition((WristServoR.getPosition() + 0.001 * Math.signum(-gamepad2.right_stick_y)));
            }


            // Airplane Launch Servo
            if (gamepad1.right_bumper) {
                AirplaneLaunchServo.setPosition(LaunchPlane * DegreeTorque);
            }


            // Intake Motor
            if (gamepad2.right_trigger != 0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.FORWARD);
                motorINTAKE.setPower(.65);
                telemetry.addLine("Currently In-taking");
            }
            else if (gamepad2.left_trigger !=0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
                motorINTAKE.setPower(.65);
                telemetry.addLine("Currently Out-taking");
            }
            else {
                motorINTAKE.setPower(0);
                telemetry.addLine("Currently Not Moving");

            }


            // Arms Movement
            if (gamepad2.a) {
                FlippyFlip.setPosition(BackboardDriverArmsFlip * Degree5Turn);
                FlooppyFloop.setPosition(BackboardDriverArmsFloop * Degree5Turn);
            }

            if (gamepad2.dpad_up) {
                FlippyFlip.setPosition(FirstSetlineBackboardDriverArmsFlip * Degree5Turn);
                FlooppyFloop.setPosition(FirstSetlineBackboardDriverArmsFloop * Degree5Turn);
            }

            if (gamepad2.y || gamepad1.dpad_down) {
                FlippyFlip.setPosition(GroundArmsFlip * Degree5Turn);
                FlooppyFloop.setPosition(GroundArmsFloop * Degree5Turn);
            }

            if (gamepad1.dpad_up) {
                FlippyFlip.setPosition(SlightlyAboveGroundDriverArmsFlip * Degree5Turn);
                FlooppyFloop.setPosition(SlightlyAboveGroundDriverArmsFloop * Degree5Turn);
            }




            // Toggle / Close & Open for the Pixel Cover

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                PixelCover_Toggle = !PixelCover_Toggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (PixelCover_Toggle) {
                PixelCoverServo.setPosition(ClosePixelCover * DegreeTorque);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {

                PixelCoverServo.setPosition(init * DegreeTorque);
            }














            // Telemetry
            telemetry.addData("WRIST SERVO R POS", WristServoR.getPosition());
            telemetry.addData("WRIST SERVO L POS", WristServoR.getPosition());
            telemetry.update();
        }
    }
}

