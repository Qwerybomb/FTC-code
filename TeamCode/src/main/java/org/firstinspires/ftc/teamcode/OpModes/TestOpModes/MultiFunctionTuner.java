package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PidController;

@TeleOp(name="Manual Tuner", group="Linear OpMode")
public class MultiFunctionTuner extends LinearOpMode {

    // target rpms
    public enum targets {
        Lowest(0),
        KindaLow(100),
        KindaHigh(200),
        Highest(300);
        final float RPM;
        targets(float RPM) {
            this.RPM = RPM;
        }
    }

    // multi-purpose mode
    public enum mode {
        ZieglerNichols,
        Manual;
    }
    mode currentMode = mode.ZieglerNichols;

    // ziegler nichols specific values
    float ultimateGain = 3f;
    float oscillationPeriod = 392.5f;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        PidController PID = new PidController(hardwareMap, 200f, true);
        Telemetry telemetry1 = FtcDashboard.getInstance().getTelemetry();
        float Proportional = 0.1f;
        float Integral = 0;
        float Derivative = 0;
        PID.setPidForTuning(Proportional, Integral, Derivative);

        while (opModeIsActive()) {

            // set the mode to either manual movement or zieglerNichols test
            if (gamepad1.left_stick_button) {
                currentMode = mode.ZieglerNichols;
            } else if (gamepad1.right_stick_button) {
                currentMode = mode.Manual;
            }

            // set the target position to reach.
            if (gamepad1.a) {
                PID.updateTarget(targets.Lowest.RPM);
            }
            if (gamepad1.b) {
                PID.updateTarget(targets.KindaLow.RPM);
            }
            if (gamepad1.x) {
                PID.updateTarget(targets.KindaHigh.RPM);
            }
            if (gamepad1.y) {
                PID.updateTarget(targets.Highest.RPM);
            }

            if (currentMode == mode.Manual) {
                Proportional += (0.005f * gamepad1.left_stick_y);
                Integral += (0.005f * gamepad1.right_stick_y);
                Derivative += (0.005f * (gamepad1.left_trigger - gamepad1.right_trigger));
                PID.setPidForTuning(Proportional, Integral, Derivative);
                telemetry1.addData("P", PID.Kp);
                telemetry1.addData("I", PID.Ki);
                telemetry1.addData("D", PID.Kd);
            }
            if (currentMode == mode.ZieglerNichols) {
                /// PID.setPidForTuning(ultimateGain * 0.6f, (1.2f * ultimateGain) / oscillationPeriod, (3 * ultimateGain * oscillationPeriod) / 40);
                PID.setPidForTuning(ultimateGain * 0.45f, (0.54f * ultimateGain) / oscillationPeriod, 0);
            }
            PID.Loop();
        }
    }
}
