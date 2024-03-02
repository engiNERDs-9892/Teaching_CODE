package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorINTAKE;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorLiftyLift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Intake Test", group="Linear Opmode")
//@Disabled
public class Intake_Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (gamepad1.left_trigger != 0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
                motorINTAKE.setPower(.65);
            }

            if (gamepad1.right_trigger != 0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.FORWARD);
                motorINTAKE.setPower(.65);
            }

            if (gamepad1.back) {
                motorINTAKE.setPower(0);
            }
        }
    }
}



