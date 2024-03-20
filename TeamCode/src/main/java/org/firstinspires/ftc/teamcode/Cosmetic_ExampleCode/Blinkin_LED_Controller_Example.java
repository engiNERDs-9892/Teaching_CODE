package org.firstinspires.ftc.teamcode.Cosmetic_ExampleCode;
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

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // This is the initialization phase where you would be able to set the initial color for the //
        // Led's during both auto and driver control                                                 //
        ///////////////////////////////////////////////////////////////////////////////////////////////

        pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
        blinkinLedDriver.setPattern(pattern);
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


        waitForStart();


        while (opModeIsActive()) {

            blinkinLedDriver.setPattern(pattern);


            //////////////////////////////////////////////
            // How to add LED to driver control actions //
            //////////////////////////////////////////////

            if (gamepad1.right_bumper) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            }
            if (gamepad1.left_bumper) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
            }
            if (gamepad1.y) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
            }
        }
    }
}

