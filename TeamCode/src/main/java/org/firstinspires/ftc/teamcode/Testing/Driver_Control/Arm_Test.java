package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;




@Autonomous(group = "advanced")
public class Arm_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop"); // RIGHT SERVO
        FlippyFlip = hardwareMap.servo.get("FlippyFlip"); // LEFT SERVO
        FlippyFlip.setDirection(Servo.Direction.REVERSE); // LEFT SERVO

        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)

        FlooppyFloop.setPosition(0);
        FlippyFlip.setPosition(0);
        waitForStart();

        while (opModeIsActive()) {
        if (gamepad1.a){
            FlooppyFloop.setPosition(0);
            FlippyFlip.setPosition(0);
        }
        if (gamepad1.b){
            FlooppyFloop.setPosition(.25);
            FlippyFlip.setPosition(.25);
        }

        }
    }

    }
