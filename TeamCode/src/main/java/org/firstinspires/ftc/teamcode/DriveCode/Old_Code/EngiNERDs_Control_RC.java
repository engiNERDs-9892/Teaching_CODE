package org.firstinspires.ftc.teamcode.DriveCode.Old_Code;

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
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorFR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorINTAKE;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorRiseyRise;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class EngiNERDs_Control_RC extends LinearOpMode {

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

        AirplaneLaunchServo.setPosition(init * DegreeTorque);
        FlooppyFloop.setPosition(init * Degree5Turn);
        FlippyFlip.setPosition(initFlip * Degree5Turn);
        WristServoL.setPosition(init * Degree5Turn);
        WristServoR.setPosition(init * Degree5Turn);

        waitForStart();

        while (opModeIsActive()) {

            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);


            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            // Variable used for Regular speed (To find the direction that the stick needs to be in) (Controller 1)
            double max;

            // The code below talks about the Y-axis (Up and Down / Forward and Backwards)

            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value

            // The code below talks about the X-axis (Left and Right) (Controller 1)

            double lateral = -gamepad1.left_stick_x; // The bottom two are inverted because motor direction is changed (Controller 1)

            // The code below talks about Z-Axis (Spinning around) (Controller 1)

            double yaw = -gamepad1.right_stick_x;

            // The code to raise and lower the Linerslides (Controller 2)

            double RaiseandLower = -gamepad2.left_stick_y;


            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Regular speed
            double leftFrontPower = (axial + lateral + yaw);
            double rightFrontPower = (axial - lateral - yaw);
            double leftBackPower = (axial - lateral + yaw);
            double rightBackPower = (axial + lateral - yaw);


            ////////////////////////////////////////////////////////////////////////////////////////////
            // use LEFT joystick to go Forward/Backwards & left/Right, and RIGHT joystick to Rotate.///
            //////////////////////////////////////////////////////////////////////////////////////////


            // This calculates the direction & power for Regular Speed
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            // sets the wheels to do whatever the calculation above tells it to do for Regular Speed
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // Setting the power for Slow Speed
            if (gamepad1.left_trigger != 0) {
                motorFL.setPower(leftFrontPower * .2);
                motorBL.setPower(leftBackPower * .2);
                motorBR.setPower(rightBackPower * .2);
                motorFR.setPower(rightFrontPower * .2);

            }

            // Setting the power for Fast Speed
            else if (gamepad1.right_trigger != 0) {

                motorFL.setPower(leftFrontPower);
                motorBL.setPower(leftBackPower);
                motorBR.setPower(rightBackPower);
                motorFR.setPower(rightFrontPower);
            }

            // Setting the power for Regular Speed
            else {
                motorFL.setPower(leftFrontPower * .6);
                motorBL.setPower(leftBackPower * .6);
                motorFR.setPower(rightFrontPower * .6);
                motorBR.setPower(rightBackPower * .6);
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




            // Toggle / Close & Open for the Right claw
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                PixelCover_Toggle = !PixelCover_Toggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (PixelCover_Toggle) {
                PixelCoverServo.setPosition(init * DegreeTorque);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                PixelCoverServo.setPosition(ClosePixelCover * DegreeTorque);
            }


            telemetry.addData("WRIST SERVO R POS", WristServoR.getPosition());
            telemetry.addData("WRIST SERVO L POS", WristServoR.getPosition());
            telemetry.update();
        }
    }
}

