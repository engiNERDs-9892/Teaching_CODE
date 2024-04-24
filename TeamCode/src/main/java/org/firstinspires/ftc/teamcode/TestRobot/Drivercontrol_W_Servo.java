package org.firstinspires.ftc.teamcode.TestRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "drive")
//@Disabled
public class Drivercontrol_W_Servo extends LinearOpMode {


    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    Servo TestServoL;


    @Override
    public void runOpMode() throws InterruptedException {

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // This is the 1st part of the initialization phase where you would be able to add in the hardware //
        // maps that are necessary and add Motor & Servo Directions                                        //
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        // This is how the Control Hub can read and use the servo (AKA: HARDWARE MAP = Needed for in order to use)
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        TestServoL = hardwareMap.servo.get("TestServoL");

        // Setting the motor Direction, so the motors rotate correctly (Default Direction = Forward)
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);


        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        boolean TestingServo_Toggle = false;

        waitForStart();

        while (opModeIsActive()) {
            ///////////////////////////////////////////////////////////////////////////////////////
            //                                      EXAMPLE                                      //
            //                                        FOR                                        //
            //                                        THE                                        //
            //                                       MOTOR                                       //
            //                                      MOVEMENT                                     //
            //                                    ROBOT CENTRIC                                  //
            //                                        WITH                                       //
            //                                 SERVO & NO ROADRUNNER                             //
            ///////////////////////////////////////////////////////////////////////////////////////

            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            /** Driver Control example is found in the Teaching_Code Repository under
             *  TeamCode -> org.firstinspires.ftc.teamcode -> DriverActions_ExampleCode ->
             *  Basic -> Movement_For_Motors_FC_Example / Movement_For_Motors_RC_Example
             */


            // Variable used for Regular speed (To find the direction that the stick needs to be in) (Controller 1)
            double max;

            // The code below talks about the Y-axis (Up and Down / Forward and Backwards)
            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value

            // The code below talks about the X-axis (Left and Right) (Controller 1)
            double lateral = -gamepad1.left_stick_x; // The bottom two are inverted because motor direction is changed (Controller 1)

            // The code below talks about Z-Axis (Spinning around) (Controller 1)
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


            // Setting the power for Regular Speed
            motorFL.setPower(leftFrontPower * .7);
            motorBL.setPower(leftBackPower * .7);
            motorFR.setPower(rightFrontPower * .7);
            motorBR.setPower(rightBackPower * .7);



            /** Toggle example is found in the Teaching_Code Repository under
             *  TeamCode -> org.firstinspires.ftc.teamcode -> DriverActions_ExampleCode -> Advanced -> TogglesForDriver.java
             */


            // Toggle / Close & Open for the Left claw
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                TestingServo_Toggle = !TestingServo_Toggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (TestingServo_Toggle) {
                TestServoL.setPosition(.99);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                TestServoL.setPosition(0.01);
            }











        }
    }
}

