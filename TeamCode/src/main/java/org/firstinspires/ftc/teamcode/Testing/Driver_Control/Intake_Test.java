package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.IntakeServo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Intake Test", group="Linear Opmode")
//@Disabled
public class Intake_Test extends LinearOpMode {

    @Override
    public void runOpMode() {
        IntakeServo = hardwareMap.servo.get("IntakeServo");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        if (gamepad1.a){
            IntakeServo.setPosition(.99);
        }

        if (gamepad1.b){
            IntakeServo.setPosition(0.01);
        }

        }
    }
}




