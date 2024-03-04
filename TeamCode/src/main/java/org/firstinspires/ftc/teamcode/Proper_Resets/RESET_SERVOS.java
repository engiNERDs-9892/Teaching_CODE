package org.firstinspires.ftc.teamcode.Proper_Resets;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(group = "advanced")
public class RESET_SERVOS extends LinearOpMode {

    public Servo servo0;
    public Servo servo1;
    public Servo servo2;
    public Servo servo3;
    public Servo servo4;
    public Servo servo5;
    @Override
    public void runOpMode() throws InterruptedException {


        servo0 = hardwareMap.servo.get("servo0");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");
        servo4 = hardwareMap.servo.get("servo4");
        servo5 = hardwareMap.servo.get("servo5");

        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)


        waitForStart();
        while (opModeIsActive()) {
          servo0.setPosition(0);
          servo1.setPosition(0);
          servo2.setPosition(0);
          servo3.setPosition(0);
          servo4.setPosition(0);
          servo5.setPosition(0);

        }
    }

    }
