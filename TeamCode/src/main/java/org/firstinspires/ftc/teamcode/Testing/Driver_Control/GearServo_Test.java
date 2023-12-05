package org.firstinspires.ftc.teamcode.Testing.Driver_Control;


import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.TeleOP_Variables.GearServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Gear Motor Test", group="Linear Opmode")
//@Disabled

public class GearServo_Test extends LinearOpMode {
   Servo SpeedServo;
    @Override
    public void runOpMode() {

        GearServo = hardwareMap.servo.get("GearServo");


        GearServo.setPosition(0);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            updateTelemetry(telemetry);

        }
    }
}




