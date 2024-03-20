package org.firstinspires.ftc.teamcode.Misc_ExampleCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "advanced", preselectTeleOp = "EngiNERDs_Control_RC_V2_BLUE")
//@Disabled
public class Preselect_Opmode_Example extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {

        }
    }
}

/**
 *  NOTE:
 *  The preselectTelop, only works if the code above is written in @Autonomous (MAIN BIT OF CODE IS IN THE () ABOVE)
 */

