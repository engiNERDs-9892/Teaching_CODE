package org.firstinspires.ftc.teamcode.AutonomusActions_ExampleCode.Basic_Actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(group = "drive")
//@Disabled
public class Movment_Based_On_ENCODER_AUTOEXAMPLE extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    int in = 45;
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

        waitForStart();
        while (opModeIsActive()) {


            ///////////////////////////////////////////////////////////////////////////////////////
            //                                      EXAMPLE                                      //
            //                                        FOR                                        //
            //                                      MOVEMENT                                     //
            //                                       DURING                                      //
            //                                        AUTO                                       //
            ///////////////////////////////////////////////////////////////////////////////////////


                // Start the movement process by resetting encoders
                motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Gives it a position to run to
                motorFL.setTargetPosition(20 * in);
                motorFR.setTargetPosition(20 * in);
                motorBL.setTargetPosition(20 * in);
                motorBR.setTargetPosition(20 * in);

                // tells it to go to the position that is set
                motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // the motor speed for Wheels
                motorFL.setPower(.5);
                motorFR.setPower(.5);
                motorBL.setPower(.5);
                motorBR.setPower(.5);


                // While loop keeps the code running until motors reach the desired position
                while (opModeIsActive() && ((motorFL.isBusy() || motorFR.isBusy()))) {
                }
                motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }

        }

    }



