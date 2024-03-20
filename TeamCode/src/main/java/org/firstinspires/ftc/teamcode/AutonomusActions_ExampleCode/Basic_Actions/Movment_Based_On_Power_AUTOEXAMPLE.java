package org.firstinspires.ftc.teamcode.AutonomusActions_ExampleCode.Basic_Actions;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(group = "drive")
//@Disabled
public class Movment_Based_On_Power_AUTOEXAMPLE extends LinearOpMode {

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


            motorFL.setPower(.6);
            motorFR.setPower(.6);
            motorBL.setPower(.6);
            motorBR.setPower(.6);

            // have the power for the motors run at .6 power for 5 seconds
            sleep(5000);

            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);

        }

    }
    }


