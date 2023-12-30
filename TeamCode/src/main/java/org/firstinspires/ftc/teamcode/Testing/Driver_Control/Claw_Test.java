package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Close;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Claw Test", group="Linear Opmode")
@Disabled
public class Claw_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        LeftClaw = hardwareMap.servo.get("LeftClaw");
        RightClaw = hardwareMap.servo.get("RightClaw");

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        // Setting the position for the Servos for Driver Control
        LeftClaw.setPosition(1);
        RightClaw.setPosition(0);

        LeftClaw.setDirection(Servo.Direction.REVERSE);


        boolean Right_Claw_Toggle = false;

        boolean Left_Claw_Toggle = false;

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);



            // Toggle / Close & Open for the Right claw
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {

                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.

                Right_Claw_Toggle = !Right_Claw_Toggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (Right_Claw_Toggle) {

                RightClaw.setPosition(Open);

            }

            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {

                RightClaw.setPosition(Close);

            }



            // Toggle / Close & Open for the Left claw
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                Left_Claw_Toggle = !Left_Claw_Toggle;
            }

            // Opens the claws after the 1st press of the bumper and alternates once pressed again
            if (Left_Claw_Toggle) {
                LeftClaw.setPosition(Open);
            }
            // Closes the claws on the 2nd press of the bumper and alternates once pressed again
            else {
                LeftClaw.setPosition(Close);
            }


        }
    }
}




