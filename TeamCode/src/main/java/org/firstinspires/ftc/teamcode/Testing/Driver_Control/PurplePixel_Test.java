package org.firstinspires.ftc.teamcode.Testing.Driver_Control;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DropPurplePixel;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PurplePixelServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(group = "advanced")
@Disabled
public class PurplePixel_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        PurplePixelServo = hardwareMap.servo.get("PurplePixelServo");
        PurplePixelServo.setDirection(Servo.Direction.REVERSE);

        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)

        PurplePixelServo.setPosition(init * DegreeTorque);
        waitForStart();

        while (opModeIsActive()) {
        if (gamepad1.a){
            PurplePixelServo.setPosition(init * DegreeTorque);
            telemetry.addLine("Ready to add Purple Pixel");
            telemetry.update();

        }
        if (gamepad1.b){
            PurplePixelServo.setPosition(DropPurplePixel * DegreeTorque);
            telemetry.addLine("Dropped Purple Pixel");
            telemetry.update();

        }

        }
    }

    }
