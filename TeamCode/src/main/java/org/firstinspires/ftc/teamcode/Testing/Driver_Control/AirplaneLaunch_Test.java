package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMountServo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Airplane Launch Test", group="Linear Opmode")
@Disabled
public class AirplaneLaunch_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");

        AirplaneLaunchServo.setDirection(Servo.Direction.FORWARD);

        AirplaneLaunchServo.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                AirplaneLaunchServo.setPosition(0);
            }
            if (gamepad1.b){
                AirplaneLaunchServo.setPosition(.35);
            }

        }
    }
}




