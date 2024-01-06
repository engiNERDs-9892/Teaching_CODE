package org.firstinspires.ftc.teamcode.Testing.Driver_Control;


import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Gear Servo Testing", group="Linear Opmode")
//@Disabled
public class GearServo_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        GearServo = hardwareMap.servo.get("GearServo");


        GearServo.setPosition(0);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (Math.abs(gamepad2.left_stick_y) >=0.5)
                GearServo.setPosition((GearServo.getPosition() + 0.0005 * Math.signum(-gamepad2.left_stick_y)));

            if (gamepad1.a || gamepad1.square){
                GearServo.setPosition(.99);
            }

            if (gamepad1.b || gamepad1.x){
                GearServo.setPosition(0);
            }
            updateTelemetry(telemetry);

        }
    }
}




