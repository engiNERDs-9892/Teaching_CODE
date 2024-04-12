package org.firstinspires.ftc.teamcode.DriverActions_ExampleCode.Basic;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group = "drive")
@Disabled
public class Rotation_For_Servo_EXAMPLE extends LinearOpMode {

    // This is how to call the Servo variable name
    public static Servo ServoName;

    @Override

    public void runOpMode() throws InterruptedException {

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // This is the 1st part of the initialization phase where you would be able to add in the hardware //
        // maps that are necessary                                                                         //
        /////////////////////////////////////////////////////////////////////////////////////////////////////



        // This is how the Control Hub can read and use the servo (AKA: HARDWARE MAP = Needed for in order to use)
        ServoName = hardwareMap.servo.get("ServoName");


        waitForStart();

        ///////////////////////////////////////////////////////
        // This is the 2nd part of the initialization phase //
        //////////////////////////////////////////////////////

        ServoName.setPosition(1);


        while (opModeIsActive()) {



            ///////////////////////////////////////////////////////////////////////////////////////
            //                                      EXAMPLE                                      //
            //                                        FOR                                        //
            //                                        THE                                        //
            //                                       SERVO                                       //
            //                                      ROTATION                                     //
            ///////////////////////////////////////////////////////////////////////////////////////

            /**
             * Note:
             * A 5 turn servos from Gobilda have a full rotation that is = 1500 Degrees
             * A Regular Gobilda servo's rotation is = 300 Degrees
             * The max Axon Servo Rotation is (150 - 355 Degrees depending on servo programmer)
             * If you set either servo to CR mode, in order to stop the CR servo from moving set the Position to (0.5)
             * If you set either servo to CR mode, in order to rotate to make it spin clockwise, set the Position to (0.51 - 0.99)
             * If you set either servo to CR mode, in order to rotate to make it spin counter clockwise, set the Position from (0 - 0.49)
             */


            // This will happen when you press the A button
            if (gamepad1.a) {

                // This rotates the servo to 0% of its rotation (0 degrees for all servos)
                ServoName.setPosition(0);
            }


            // When you press the right trigger on gamepad 1
            if (gamepad1.right_trigger != 0){

                // This rotates the servo to 50% of its rotation (150 Degrees for regular and a 5 turn = 750)
                ServoName.setPosition(.5);
            }



            // When you move the gamepad 1's right stick on the y-axis
            if (Math.abs(gamepad1.right_stick_y) >= 0.5) {

                // This rotates the servo incrementally, (by 0.0005 * Value of joystick (0-1)) While the joystick
                // is pushed up / pushed down
                ServoName.setPosition((ServoName.getPosition() + 0.0005 * Math.signum(-gamepad1.right_stick_y)));
            }
        }
    }
}

