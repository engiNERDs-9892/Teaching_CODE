package org.firstinspires.ftc.teamcode.DriverActions.Advanced;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group = "drive")
//@Disabled
public class TogglesForDrivers extends LinearOpMode {

    public static Servo Servoname;

    @Override

    public void runOpMode() throws InterruptedException {

        Servoname = hardwareMap.servo.get("Servoname");


        boolean PixelCover_Toggle = false;

        waitForStart();


        while (opModeIsActive()) {

        }
    }
}

