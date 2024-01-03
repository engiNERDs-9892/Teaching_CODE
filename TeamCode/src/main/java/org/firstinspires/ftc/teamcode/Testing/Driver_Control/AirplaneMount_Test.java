package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMountServo;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Airplane Mount Test", group="Linear Opmode")
@Disabled
public class AirplaneMount_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        AirplaneMountServo = hardwareMap.servo.get("AirplaneMountServo");

        AirplaneMountServo.setDirection(Servo.Direction.FORWARD);

        AirplaneMountServo.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                AirplaneMountServo.setPosition(0);
            }
            if (gamepad1.b){
                AirplaneMountServo.setPosition(.35);
            }

        }
    }
}




