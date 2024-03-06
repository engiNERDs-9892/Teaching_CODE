package org.firstinspires.ftc.teamcode.Testing.Driver_Control;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.LaunchPlane;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(group = "advanced")
@Disabled
public class AirplaneLaunch_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        AirplaneLaunchServo = hardwareMap.servo.get("AirplaneLaunchServo");
        AirplaneLaunchServo.setDirection(Servo.Direction.REVERSE);

        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)

        AirplaneLaunchServo.setPosition(init * DegreeTorque);
        waitForStart();

        while (opModeIsActive()) {
        if (gamepad1.a){
            AirplaneLaunchServo.setPosition(init * DegreeTorque);
            telemetry.addLine("Ready to Launch the Plane");
            telemetry.update();
        }
        if (gamepad1.b){
            AirplaneLaunchServo.setPosition(LaunchPlane * DegreeTorque);
            telemetry.addLine("Launched the Plane");
            telemetry.update();
        }

        }
    }

    }
