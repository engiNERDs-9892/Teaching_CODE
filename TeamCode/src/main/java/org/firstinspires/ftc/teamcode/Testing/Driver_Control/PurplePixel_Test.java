package org.firstinspires.ftc.teamcode.Testing.Driver_Control;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DropPurplePixel;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.InitPlane;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.InitPurplePixel;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.LaunchPlane;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.PurplePixelServo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(group = "advanced")
public class PurplePixel_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        PurplePixelServo = hardwareMap.servo.get("PurplePixelServo");
        PurplePixelServo.setDirection(Servo.Direction.REVERSE);

        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)

        PurplePixelServo.setPosition(InitPurplePixel * DegreeTorque);
        waitForStart();

        while (opModeIsActive()) {
        if (gamepad1.a){
            PurplePixelServo.setPosition(InitPurplePixel * DegreeTorque);
        }
        if (gamepad1.b){
            PurplePixelServo.setPosition(DropPurplePixel * DegreeTorque);
        }

        }
    }

    }
