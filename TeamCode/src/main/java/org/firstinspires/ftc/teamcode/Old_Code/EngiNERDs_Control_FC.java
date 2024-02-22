package org.firstinspires.ftc.teamcode.Old_Code;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.ShootPlane;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorBL;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorBR;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorFL;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorFR;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorINTAKE;
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
        boolean IntakeToggle = false;

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
                WristServo.setPosition((WristServo.getPosition() + 0.0005 * Math.signum(-gamepad2.right_stick_y)));
            }







            // Airplane Launch Servo
            if (gamepad1.right_bumper) {
                AirplaneLaunchServo.setPosition(ShootPlane * DegreeTorque);
            }






            // Toggle / Raise and Lower for the Arms
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                IntakeToggle = !IntakeToggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (IntakeToggle) {
                motorINTAKE.setPower(.65);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                motorINTAKE.setPower(-.65);
            }
















            // Telemetry
            telemetry.addData("LEFT LS POS", motorLiftyLift.getCurrentPosition());
            telemetry.addData("RIGHT LS POS", motorRiseyRise.getCurrentPosition());
            telemetry.addData("WRIST SERVO POS", WristServo.getPosition());
            telemetry.addData("LEFT ARM POS", FlooppyFloop.getPosition());
            telemetry.addData("RIGHT ARM POS", FlippyFlip.getPosition());
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
