package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Arm Test", group="Linear Opmode")
@Disabled
public class Arm_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");

        FlooppyFloop.setPosition(1);
        FlippyFlip.setPosition(0);

        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        FlooppyFloop.setDirection(Servo.Direction.REVERSE);


        boolean Arm_Toggle = false;


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);



            // Toggle / Close & Open for the Left claw
            if (currentGamepad2.a && !previousGamepad2.a) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                Arm_Toggle = !Arm_Toggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (Arm_Toggle) {
                FlooppyFloop.setPosition(.5);
                FlippyFlip.setPosition(.5);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                FlooppyFloop.setPosition(.02);
                FlippyFlip.setPosition(.98);
            }

            updateTelemetry(telemetry);

        }
    }
}




