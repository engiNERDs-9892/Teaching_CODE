package org.firstinspires.ftc.teamcode.Misc_ExampleCode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(group = "drive")
//@Disabled
public class Blinkin_LED_Controller_Example extends LinearOpMode {
    public static RevBlinkinLedDriver blinkinLedDriver; // Variable name of the Brains of the LED's
    public static RevBlinkinLedDriver.BlinkinPattern pattern; // Variable name of how to change th color

    @Override

    public void runOpMode() throws InterruptedException {

        //////////////////////////////////////////////////////////////////////////////////////////
        // This is the 1st part of the initialization phase where you would be able to set the  //
        // initial color for the Led's during both auto and driver control                      //
        //////////////////////////////////////////////////////////////////////////////////////////

        pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;  // Choosing the starting color

        blinkinLedDriver.setPattern(pattern); // Tells the LED's to become the color that was chosen to start with

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"); // Hardware map for the brain of the LED's



        waitForStart();

        ///////////////////////////////////////////////////////
        // This is the 2nd part of the initialization phase //
        //////////////////////////////////////////////////////

        while (opModeIsActive()) {

            blinkinLedDriver.setPattern(pattern); // Continues to be the color that was set from the initialization phase


            //////////////////////////////////////////////
            // How to add LED to driver control actions //
            //////////////////////////////////////////////

            if (gamepad1.right_bumper) {

                // Changes the color of the LED's to GREEN if right bumper is pressed on gamepad 1
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            }
            if (gamepad1.left_bumper) {

                // Changes the color of the LED's to DARK_BLUE if left bumper is pressed on gamepad 1
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
            }
            if (gamepad1.y) {

                // Changes the color of the LED's to DARK_RED if y is pressed on gamepad 1
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
            }


            /**
             * NOTE:
             *  You can change the color of the LED's in the same way as you do in driver control for the
             *  Autonomous Period. You would need an if statement along with either the sensor that is pressed / is true,
             *  or you would need to do a FSM (Finite State Machine) and tell the LED's to change depending on the state
             *  or phase that the Autonomous is
             */
        }
    }
}

