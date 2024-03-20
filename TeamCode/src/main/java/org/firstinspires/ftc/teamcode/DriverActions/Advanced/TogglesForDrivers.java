package org.firstinspires.ftc.teamcode.DriverActions.Advanced;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group = "drive")
//@Disabled
public class TogglesForDrivers extends LinearOpMode {

    // This is how to call the Servo variable name
    public static Servo Servoname;

    @Override

    public void runOpMode() throws InterruptedException {
        // This is how the Control Hub can read and use the servo (AKA: HARDWARE MAP = Needed for in order to use)
        Servoname = hardwareMap.servo.get("Servoname");

        // Variable names that are used to determine what the current game pad input is (For both game pad 1 & 2)
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // Variable names that are used to determine what the current game pad input is (For both game pad 1 & 2)
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        boolean Example_Toggle = false;             // A true or false statement that tells the

        waitForStart();


        while (opModeIsActive()) {

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);         // This stores the last values that were pressed from gamepad 1
            currentGamepad2.copy(gamepad2);         // This stores the last values that were pressed from gamepad 2

            // Used values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1); // This checks and make sures that it has the proper position stored
            previousGamepad2.copy(currentGamepad2); // This checks and make sures that it has the proper position stored







            ///////////////////////////////////////////////////////////////////////////////////////
            //                                      EXAMPLE                                      //
            //                                        FOR                                        //
            //                                        THE                                        //
            //                                       TOGGLE                                      //
            ///////////////////////////////////////////////////////////////////////////////////////

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {


                Example_Toggle = !Example_Toggle;


            }


            if (Example_Toggle) {
                Servoname.setPosition(0);
            }



            else {
                // This will trigger after the 2nd time the right bumper is pressed on Gamepad 2
                Servoname.setPosition(.5);
            }


             /**
             * Note:
             * Whatever is in the else statement, is the position that the Servo will initialize to
             * Whatever is in the else statement, is the power a Motor will have during initialization.
             */
        }
    }
}

