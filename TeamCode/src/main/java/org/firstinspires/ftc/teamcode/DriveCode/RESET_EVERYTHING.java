package org.firstinspires.ftc.teamcode.DriveCode;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeAirplane;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeWrist;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorBL;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorBR;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorFL;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorFR;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorRiseyRise;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables;

@TeleOp(name="RESET_HARDWARE", group="Linear Opmode")
//@Disabled
public class RESET_EVERYTHING extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        new EngiNERDs_Variables(hardwareMap);


        // Declare our IMU (Inertial Motion Unit)
        // Make sure your ID's match your configuration
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // Setting the motor Direction, so the motors or servos rotate correctly (Default Direction = Forward)

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorRiseyRise.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLiftyLift.setDirection(DcMotorSimple.Direction.FORWARD);

        // this sets the servos in the proper direction
        LeftClaw.setDirection(Servo.Direction.REVERSE);
        RightClaw.setDirection(Servo.Direction.REVERSE);
        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);
        GearServo.setDirection(Servo.Direction.REVERSE);

        // Adjust the orientation parameters to match your robot (Adjust which way the Control Hub is facing)
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        // Resets the Encoder Position to 0 so that we can use Encoders for our Driver Control instead
        // of Magnetic Limit Switches
        motorRiseyRise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftyLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Tells the motors to Run using those specific encoders
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftyLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setting the motor Power for Driver Control
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorRiseyRise.setPower(0);
        motorLiftyLift.setPower(0);

        // this call sets the servos during initialization
        // this call sets the servos during initialization
        LeftClaw.setPosition(300 * DegreeClaw); // Closes
        RightClaw.setPosition(0 * DegreeClaw); // Closes
        GearServo.setPosition(0 * DegreeWrist);
        AirplaneServo.setPosition(300 * DegreeAirplane);


        imu.initialize(parameters);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox / PS4 controllers.
            if (gamepad1.back) {
                imu.resetYaw();
            }
        }
    }
}