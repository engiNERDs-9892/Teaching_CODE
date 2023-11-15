package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlooppyFloop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Arm_Test", group="Linear Opmode")
//@Disabled

public class Arm_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");

        FlooppyFloop.setPosition(1);
        FlippyFlip.setPosition(0);

        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Statement = If you are pushing up on the right joystick, then rotate the arms behind the robot
            if(Math.abs(gamepad2.right_stick_y) <= -0.5) {

                // This rotates the arms Clockwise so that the arms rotate behind the robot (Facing the backboard idealy)
                // FlippyFlip adds to its current position due to the value starting at zero
                FlippyFlip.setPosition((FlippyFlip.getPosition() + 0.0005 * Math.signum(gamepad2.right_stick_y)));

                // FloopyFloop subtracts from its current position due to the value starting at One
                FlooppyFloop.setPosition((FlooppyFloop.getPosition() - 0.0005 * Math.signum(gamepad2.right_stick_y)));
            }

            // Statement = If you are pushing down on the right joystick, then rotate the arms to in front of the robot
            if(Math.abs(gamepad2.right_stick_y) >= 0.5) {

                // This rotates the arms Counter Clockwise so that the arms rotate in front of the robot
                // FlippyFlip subtracts from its current position due to the value starting at zero
                FlippyFlip.setPosition((FlippyFlip.getPosition() - 0.0005 * Math.signum(gamepad2.right_stick_y)));

                // FloopyFloop adds to its current position due to the value starting at One
                FlooppyFloop.setPosition((FlooppyFloop.getPosition() + 0.0005 * Math.signum(gamepad2.right_stick_y)));
            }


            telemetry.addData("Arm Position", FlippyFlip.getPosition());
            telemetry.addData("Arm Position", FlooppyFloop.getPosition());
            updateTelemetry(telemetry);

        }
    }
}




