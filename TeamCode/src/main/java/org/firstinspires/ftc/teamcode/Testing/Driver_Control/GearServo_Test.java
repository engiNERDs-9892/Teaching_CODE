package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.GearServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Gear Servo Test", group="Linear Opmode")
//@Disabled

public class GearServo_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        GearServo = hardwareMap.servo.get("GearServo");

        GearServo.setPosition(0);

        GearServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (gamepad2.left_stick_y <= -0.5) {
                // This rotate the gear Clockwise so that the wrist joint rotates to the back of the Robot
                // by adding to its current position
                GearServo.setPosition((GearServo.getPosition() + 0.0005 * (gamepad2.left_stick_y)));
            }
            if (gamepad2.left_stick_y >= 0.5) {
                GearServo.setPosition((GearServo.getPosition() + 0.0005 * (gamepad2.left_stick_y)));
            }


            telemetry.addData("GearServo Position", GearServo.getPosition());
            updateTelemetry(telemetry);

        }
    }
}




