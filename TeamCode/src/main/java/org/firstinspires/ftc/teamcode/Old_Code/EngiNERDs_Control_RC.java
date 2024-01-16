package org.firstinspires.ftc.teamcode.Old_Code;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMountServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMount_Rotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.BackboardArmRotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Close;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeAirplane;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeArm;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeWrist;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GroundArmRotate;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateGround;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateStack;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorBL;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorBR;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorFL;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorFR;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorRiseyRise;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables;


@TeleOp(name="EngiNERDs Control RC", group="Linear Opmode")
@Disabled

public class EngiNERDs_Control_RC extends LinearOpMode {
    @Override
    public void runOpMode() {

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        boolean Right_Claw_Toggle = false;

        boolean Left_Claw_Toggle = false;

        boolean Arm_Toggle = false;

        boolean AirplaneMount_Toggle = false;

        // Hardware Map
        new EngiNERDs_Variables(hardwareMap);
        FlooppyFloop.setPosition(GroundArmRotate * DegreeArm);
        FlippyFlip.setPosition(GroundArmRotate * DegreeArm);
        GearServo.setPosition(WristRotateGround * DegreeWrist);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            // Encoder tracker for the LS so it doesn't go over the limit / to high or low
            int LiftyLiftPos = motorLiftyLift.getCurrentPosition();
            int RiseyRisePos = motorRiseyRise.getCurrentPosition();

            // Variable used for Regular speed (To find the direction that the stick needs to be in)
            double max;

            // Joystick Values for the Linear slides
            double RaiseandLower = -gamepad2.right_stick_y;

            // The code below talks about the Y-axis (Up and Down / Forward and Backwards)

            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value

            // The code below talks about the X-axis (Left and Right)

            double lateral = -gamepad1.left_stick_x; // The bottom two are inverted because motor direction is changed

            // The code below talks about Z-Axis (Spinning around)

            double yaw = -gamepad1.right_stick_x;


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


            // If you are trying to raise the linear Slide
            // Then raise the linear slides!
            if (RaiseandLower < -0.05) {
                motorRiseyRise.setPower(RaiseandLower * 1);
                motorLiftyLift.setPower(RaiseandLower * 1);
            }


            // if you are trying to lower the linear Slide
            // Then lower the linear slides!
            if (RaiseandLower > 0.05) {
                motorRiseyRise.setPower(RaiseandLower * 1);
                motorLiftyLift.setPower(RaiseandLower * 1);
            }

            // If you are not pushing on the joystick the power = 0
            // This is mainly to prevent stick drift
            if ((RaiseandLower >= -0.05) && (RaiseandLower <= 0.05)) {
                motorRiseyRise.setPower(0);
                motorLiftyLift.setPower(0);
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
                FlooppyFloop.setPosition(BackboardArmRotate * DegreeArm);
                FlippyFlip.setPosition(BackboardArmRotate * DegreeArm);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                FlooppyFloop.setPosition(GroundArmRotate * DegreeArm);
                FlippyFlip.setPosition(GroundArmRotate * DegreeArm);
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


            // if (gamepad1.right_bumper) {
            //    AirplaneLaucherServo.setPosition(AirplaneLauncher_Rotate * DegreeAirplane);
            // }





            if (Math.abs(gamepad2.left_stick_y) >= 0.5) {
                GearServo.setPosition((GearServo.getPosition() + 0.0005 * Math.signum(-gamepad2.left_stick_y)));
            }



            telemetry.addData("Left Claw Position", LeftClaw.getPosition());
            telemetry.addData("Left Claw Position", RightClaw.getPosition());
            telemetry.addData("Gear Servo Position", GearServo.getPosition());
            updateTelemetry(telemetry);
        }

    }
}
