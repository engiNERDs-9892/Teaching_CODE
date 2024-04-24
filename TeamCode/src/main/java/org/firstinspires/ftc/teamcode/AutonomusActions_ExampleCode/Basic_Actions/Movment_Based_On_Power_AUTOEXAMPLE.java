package org.firstinspires.ftc.teamcode.AutonomusActions_ExampleCode.Basic_Actions;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(group = "drive")
@Disabled
public class Movment_Based_On_Power_AUTOEXAMPLE extends LinearOpMode {

    ////////////////////////////////////////////////////////////
    // Creating the Variables for the robot to use            //
    // such as motors, servos, led controller, variables ETC. //
    ////////////////////////////////////////////////////////////

    // Variables used in order to control the motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    @Override

    public void runOpMode() throws InterruptedException {

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // This is the 1st part of the initialization phase where you would be able to add in the hardware //
        // maps that are necessary and add Motor Directions                                                //
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        // This is how the Control Hub can read and use the motors (AKA: HARDWARE MAP = Needed for in order to use)
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        // Setting the motor Direction, so the motors rotate correctly (Default Direction of a motor = Forward)
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();


            ///////////////////////////////////////////////////////////////////////////////////////
            //                                      EXAMPLE                                      //
            //                                        FOR                                        //
            //                                      MOVEMENT                                     //
            //                                       DURING                                      //
            //                                        AUTO                                       //
            ///////////////////////////////////////////////////////////////////////////////////////


            // When the opmode starts, set the power of the motors to 60%
            motorFL.setPower(.6);
            motorFR.setPower(.6);
            motorBL.setPower(.6);
            motorBR.setPower(.6);

            // have the power for the motors run at .6 power for 5 seconds
            sleep(5000);

            // After the 5 second wait, change the power of the motors from 60% power to 0% power, stopping the motors
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);

        }
        /**
         * NOTE:
         * We can change the direction of the robot after the power is set to 0 and repeat the same actions using the set
         * motor Direction up above, but it is recommended that you first change the direction of the motors,
         * add a sleep command, THEN add the motor power after the sleep command. Otherwise, if you put it before the
         * change of directions, the motors will go the same direction as the previous step with the new power added
         * to the motors.
         */
    }



