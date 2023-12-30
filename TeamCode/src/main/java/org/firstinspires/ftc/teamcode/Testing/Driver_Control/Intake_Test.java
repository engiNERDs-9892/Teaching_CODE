package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.IntakeServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Intake Test", group="Linear Opmode")
//@Disabled
public class Intake_Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeServo = hardwareMap.servo.get("GearServo");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
        if (gamepad1.a){
            IntakeServo.setPosition(0.99);
        }

        if (gamepad1.b){
            IntakeServo.setPosition(0.01);
        }

        telemetry.addData("servo Pos", IntakeServo.getPosition());
        }
    telemetry.update();
    }
}




