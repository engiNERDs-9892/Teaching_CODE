package org.firstinspires.ftc.teamcode.DriveCode;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMountServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Close;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorBL;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorBR;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorFL;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorFR;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorRiseyRise;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.slideySlideMax;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.slideySlideMin;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables;

@TeleOp(name="EngiNERDs Control FC", group="Linear Opmode")
@Disabled
public class EngiNERDs_Control_FC extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        new EngiNERDs_Variables(hardwareMap);

        // Declare our IMU (Inertial Motion Unit)
        // Make sure your ID's match your configuration
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot (Adjust which way the Control Hub is facing)
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));


        // Toggels so that the Claws can be opened and closed using the same button
        boolean Right_Claw_Toggle = false;

        boolean Left_Claw_Toggle = false;

        boolean Arm_Toggle = false;

        boolean Hook_Toggle = false;

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double RaiseandLower = -gamepad2.right_stick_y;


            // A way to store values that gamepad enters
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Calculates the Encoder values so the Linear Slides will stop once they reach a certain point
            int LiftyLiftPos = motorLiftyLift.getCurrentPosition();
            int RiseyRisePos = motorRiseyRise.getCurrentPosition();

            // Variables used to control the movement of the robot
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Calculates the current heading of the robot
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // If the Right Trigger is pressed set the Values of the motor to 100% power
            if (gamepad1.right_trigger != 0) {
                motorFL.setPower(frontLeftPower);
                motorBL.setPower(backLeftPower);
                motorFR.setPower(frontRightPower);
                motorBR.setPower(backRightPower);
            }

            // If the Left Trigger is pressed set the Values of the motor to 30% power
            if (gamepad1.left_trigger != 0) {
                motorFL.setPower(frontLeftPower * .3);
                motorBL.setPower(backLeftPower * .3);
                motorFR.setPower(frontRightPower * .3);
                motorBR.setPower(backRightPower * .3);
            }

            // If No Trigger is pressed set the Values of the motor to 70% (Base value for motors)
            else {
                motorFL.setPower(frontLeftPower * .7);
                motorBL.setPower(backLeftPower * .7);
                motorFR.setPower(frontRightPower * .7);
                motorBR.setPower(backRightPower * .7);

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

            if (Math.abs(gamepad2.left_stick_y) >= 0.5) {
                GearServo.setPosition((GearServo.getPosition() + 0.005 * Math.signum(gamepad2.left_stick_y)));
            }


            if (gamepad1.a) {
                AirplaneMountServo.setPosition(0.35);
            }


            telemetry.addData("LiftyLift Position", LiftyLiftPos);
            telemetry.addData("RiseyRise Position", RiseyRisePos);
            telemetry.addData("Left Claw Position", LeftClaw.getPosition());
            telemetry.addData("Left Claw Position", RightClaw.getPosition());
            telemetry.addData("Gear Servo Position", GearServo.getPosition());
            updateTelemetry(telemetry);


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox / PS4 controllers.
            if (gamepad1.back) {
                imu.resetYaw();
            }
        }
    }
}
