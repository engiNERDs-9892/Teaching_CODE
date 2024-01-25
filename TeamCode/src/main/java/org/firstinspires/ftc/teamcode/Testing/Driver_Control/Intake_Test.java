package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.IntakeServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorINTAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Intake Test", group="Linear Opmode")
//@Disabled
public class Intake_Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        IntakeServo = hardwareMap.servo.get("IntakeServo");

        motorINTAKE = hardwareMap.dcMotor.get("motorINTAKE");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                IntakeServo.setPosition(0.99);
            }

            if (gamepad1.b) {
                IntakeServo.setPosition(0.01);
            }


            if (gamepad1.left_trigger != 0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
                motorINTAKE.setPower(1);
            }

            if (gamepad1.right_trigger != 0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.FORWARD);
                motorINTAKE.setPower(1);
            }

            if (gamepad1.back) {
                motorINTAKE.setPower(0);
            }
            telemetry.update();
        }
    }
}



