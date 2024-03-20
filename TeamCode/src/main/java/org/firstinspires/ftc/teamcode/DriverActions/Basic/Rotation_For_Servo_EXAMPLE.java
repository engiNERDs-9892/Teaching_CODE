package org.firstinspires.ftc.teamcode.DriverActions.Basic;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group = "drive")
//@Disabled
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


            if (gamepad1.a) {
                // This will trigger after some presses A
                ServoName.setPosition(0);
            }



            if (gamepad1.right_trigger != 0){
                // This will trigger when the right trigger is pushed down
                ServoName.setPosition(.5);
            }
        }
    }
}

