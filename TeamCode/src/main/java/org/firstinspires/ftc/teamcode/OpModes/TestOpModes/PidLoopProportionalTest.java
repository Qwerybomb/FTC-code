package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.PidController;

@TeleOp(name="PID proportional test", group="Linear OpMode")
public class PidLoopProportionalTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        PidController PID = new PidController(hardwareMap, 200f, true);
        float proportional = 1;

        while (opModeIsActive()) {
            proportional += (0.005f * gamepad1.left_stick_y);
            PID.setPidForTuning(proportional,0,0);
            telemetry.addData("P", PID.Ki);
            telemetry.addData("I", PID.Ki);
            telemetry.addData("D", PID.Kd);
            PID.Loop();
        }
    }
}
