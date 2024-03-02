package org.firstinspires.ftc.teamcode.Testing.Driver_Control;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(group = "advanced")
public class Arm_Test extends LinearOpMode {

    public Servo servo0;
    public Servo servo1;
    public Servo servo2;
    public Servo servo3;
    public Servo servo4;
    public Servo servo5;
    public Servo servo6;
    @Override
    public void runOpMode() throws InterruptedException {


        servo3 = hardwareMap.servo.get("servo3");
        servo4 = hardwareMap.servo.get("servo4");
        servo4.setDirection(Servo.Direction.REVERSE);

        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)

        servo3.setPosition(0);
        servo4.setPosition(0);
        waitForStart();

        while (opModeIsActive()) {
        if (gamepad1.a){
            servo3.setPosition(0);
            servo4.setPosition(0);
        }
        if (gamepad1.b){
            servo3.setPosition(.25);
            servo4.setPosition(.25);
        }

        }
    }

    }
