package org.firstinspires.ftc.teamcode.Testing.Driver_Control;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ClosePixelCover;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.LaunchPlane;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PixelCoverServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorINTAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group = "advanced")
public class PixelCover_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        PixelCoverServo = hardwareMap.servo.get("PixelCoverServo");
        motorINTAKE = hardwareMap.dcMotor.get("motorINTAKE");

        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)

        PixelCoverServo.setPosition(init * DegreeTorque);
        waitForStart();

        while (opModeIsActive()) {
        if (gamepad1.a){
            PixelCoverServo.setPosition(init * DegreeTorque);
            telemetry.addLine("Open");
            telemetry.update();
        }
        if (gamepad1.b){
            PixelCoverServo.setPosition(ClosePixelCover * DegreeTorque);
            telemetry.addLine("Closed");
            telemetry.update();
        }

            if (gamepad1.left_trigger != 0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
                motorINTAKE.setPower(.65);

                telemetry.addLine("OUTTAKING PIXELS CURRENTLY");
                telemetry.update();
            }

            if (gamepad1.right_trigger != 0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.FORWARD);
                motorINTAKE.setPower(.65);
                telemetry.addLine("INTAKING PIXELS CURRENTLY");
                telemetry.update();
            }

            if (gamepad1.back) {
                motorINTAKE.setPower(0);
                telemetry.addLine("NOT DOING ANYTHING");
                telemetry.update();
            }
        }
    }

    }
