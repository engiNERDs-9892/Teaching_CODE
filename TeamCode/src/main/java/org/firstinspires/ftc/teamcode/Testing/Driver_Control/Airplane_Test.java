package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneServo;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Airplane_Test", group="Linear Opmode")
@Disabled
public class Airplane_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        AirplaneServo = hardwareMap.servo.get("AirplaneServo");

        AirplaneServo.setPosition(1);

        AirplaneServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
               AirplaneServo.setPosition(0);
            }
            if (gamepad1.b){
                AirplaneServo.setPosition(.35);
            }

        }
    }
}




