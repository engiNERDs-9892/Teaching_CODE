package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class pid_LS extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static  int target;

    private final double ticks_in_degree = 700 / 180.0;
    private DcMotorEx lift;

    @Override
    public void init() {
 controller = new PIDController(p,i,d);
 telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
 lift = hardwareMap.get(DcMotorEx.class, "lift");
 lift.setDirection(DcMotorSimple.Direction.REVERSE);
 lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 controller.setPID(p,i,d);
    }

    @Override
    public void loop() {
int liftPos = lift.getCurrentPosition();
double pid = controller.calculate(liftPos,target);

double power= pid;
lift.setPower(power);
 telemetry.addData("pos",liftPos);
 telemetry.addData("target",target);
 telemetry.update();
    }
}
