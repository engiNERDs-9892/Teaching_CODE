package org.firstinspires.ftc.teamcode.TestRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group = "drive")
//@Disabled
public class Drivercontrol_W_Servo extends LinearOpMode {

    // This is how to call the Motor variable name

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    Servo TestServoL;

    @Override

    public void runOpMode() throws InterruptedException {

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // This is the 1st part of the initialization phase where you would be able to add in the hardware //
        // maps that are necessary and add Motor Directions                                                //
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        // Setting the motor Direction, so the motors rotate correctly (Default Direction = Forward)
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);


        // This is how the Control Hub can read and use the servo (AKA: HARDWARE MAP = Needed for in order to use)
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        TestServoL = hardwareMap.servo.get("TestServoL");





        // Variable names that are used to determine what the current game pad input is (For both game pad 1 & 2)
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // Variable names that are used to determine what the current game pad input is (For both game pad 1 & 2)
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        boolean Example_Toggle = false;             // A true or false statement that tells the Toggle what state it is in

        waitForStart();

        ///////////////////////////////////////////////////////
        // This is the 2nd part of the initialization phase //
        //////////////////////////////////////////////////////



        while (opModeIsActive()) {



            ///////////////////////////////////////////////////////////////////////////////////////
            //                                      EXAMPLE                                      //
            //                                        FOR                                        //
            //                                        THE                                        //
            //                                       MOTOR                                       //
            //                                      MOVEMENT                                     //
            //                                    ROBOT CENTRIC                                  //
            //                                        W/O                                        //
            //                                     ROADRUNNER                                    //
            ///////////////////////////////////////////////////////////////////////////////////////


            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);         // This stores the last values that were pressed from gamepad 1
            currentGamepad2.copy(gamepad2);         // This stores the last values that were pressed from gamepad 2

            // Used values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1); // This checks and make sures that it has the proper position stored
            previousGamepad2.copy(currentGamepad2); // This checks and make sures that it has the proper position stored




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




            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {

                // Basically this is saying that when the _______ is pressed, change the value from
                // false to true and vice versa
                Example_Toggle = !Example_Toggle;


            }
            if (Example_Toggle) {
                // This will trigger after the 1st time the _______ is pressed on Gamepad 2
                TestServoL.setPosition(0);
            }
            else {
                // This will trigger after the 2nd time the ________ is pressed on Gamepad 2
                TestServoL.setPosition(.99);
            }

        }
    }
}

