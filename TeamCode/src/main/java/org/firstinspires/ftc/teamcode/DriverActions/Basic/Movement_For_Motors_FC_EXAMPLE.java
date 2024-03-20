package org.firstinspires.ftc.teamcode.DriverActions.Basic;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp(group = "drive")
//@Disabled
public class Movement_For_Motors_FC_EXAMPLE extends LinearOpMode {

    // This is how to call the Motor variable name

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
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        // This helps the robot know the proper orientation, rather than adjust the orientation of the robot manually
        // by resetting the heading
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        ///////////////////////////////////////////////////////
        // This is the 2nd part of the initialization phase //
        //////////////////////////////////////////////////////

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            ///////////////////////////////////////////////////////////////////////////////////////
            //                                      EXAMPLE                                      //
            //                                        FOR                                        //
            //                                        THE                                        //
            //                                       MOTOR                                       //
            //                                      MOVEMENT                                     //
            //                                    FIELD CENTRIC                                  //
            //                                        W/O                                        //
            //                                     ROADRUNNER                                    //
            ///////////////////////////////////////////////////////////////////////////////////////



            double y = -gamepad1.left_stick_y;    // The code below talks about the Y-axis (Up and Down / Forward and Backwards)
            double x = gamepad1.left_stick_x;     // The code below talks about the X-axis (Left and Right) (Controller 1)
            double rx = gamepad1.right_stick_x;   // The code below talks about Z-Axis (Spinning around) (Controller 1)


            // This calculates the current robot heading in radians
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double motorFLPower = (rotY + rotX + rx) / denominator;
            double motorFRPower = (rotY - rotX - rx) / denominator;
            double motorBLPower = (rotY - rotX + rx) / denominator;
            double motorBRPower = (rotY + rotX - rx) / denominator;

            // This gives the motors power based on the calculations, with a 70% power cap included
            // The  * .7 is how the power is capped

            motorFL.setPower(motorFLPower * .7);
            motorFR.setPower(motorFRPower * .7);
            motorBL.setPower(motorBLPower * .7);
            motorBR.setPower(motorBRPower * .7);


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }
        }

    }
    }


