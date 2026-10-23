package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.*;

import org.firstinspires.ftc.teamcode.Modules.PidController;

@TeleOp(name="PID tuning", group="Linear OpMode")
public class PidLoopTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        PidController PID = new PidController(hardwareMap, 200f, true);

        // actual ultimate gain is 3.0 -> 3.1
        float ultimate_gain = 3.9f;
        float oscillation_period = 333f;

        // PID.setPidForTuning(0,0,0);

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                ultimate_gain += 0.05f;
            }
            if (gamepad1.bWasPressed()) {
                ultimate_gain -= 0.05f;
            }
            PID.setPidForTuning(ultimate_gain * 0.6f, (1.2f * ultimate_gain) / oscillation_period, (3 * ultimate_gain * oscillation_period) / 40);
            telemetry.addData("P", PID.Ki);
            telemetry.addData("I", PID.Ki);
            telemetry.addData("D", PID.Kd);
            telemetry.update();

            PID.Loop();
        }
    }
}
