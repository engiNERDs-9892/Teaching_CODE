package org.firstinspires.ftc.teamcode.DriveCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Gear Servo Test", group="Linear Opmode")
//@Disabled

public class GearServo_Test extends LinearOpMode {

    Servo GearServo;
    @Override
    public void runOpMode() {

        GearServo = hardwareMap.servo.get("GearServo");

        GearServo.setPosition(0);



        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            while (gamepad2.left_stick_y <= -0.5) {
                GearServo.setPosition((GearServo.getPosition() + 0.01));
            }
            while (gamepad2.left_stick_y >= 0.5) {
                GearServo.setPosition((GearServo.getPosition() - 0.01));
            }



            updateTelemetry(telemetry);
        }
    }
}




