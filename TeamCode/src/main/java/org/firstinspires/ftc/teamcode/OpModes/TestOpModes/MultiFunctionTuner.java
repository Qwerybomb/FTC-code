package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID.Gains;
import org.firstinspires.ftc.teamcode.PID.PidController;
import org.firstinspires.ftc.teamcode.Modules.PidSingleMotor;

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

    public enum manualTunedGains {
        gainOne(0.14f, 0.06047f, 0.00142f);

        final float Kp;
        final float Ki;
        final float Kd;

        manualTunedGains(float Kp, float Ki ,float Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }
    }

    // multi-purpose mode
    public enum mode {
        ZieglerNichols,
        Manual;
    }
    mode currentMode = mode.Manual;

    // ziegler nichols specific values
    float ultimateGain = 3f;
    float oscillationPeriod = 392.5f;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        PidController PID = new PidSingleMotor(hardwareMap, 200f, 50);
        Telemetry telemetry1 = FtcDashboard.getInstance().getTelemetry();
        float Proportional = 0.0001f;
        float Integral = 0;
        float Derivative = 0;
        PID.gains = new Gains(0,0,0);;

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
                Proportional += (0.000005f * gamepad1.left_stick_y);
                Integral += (0.00005f * gamepad1.right_stick_y);
                Derivative += (0.00005f * (gamepad1.left_trigger - gamepad1.right_trigger));

                PID.gains.Reconstruct(Proportional, Integral, Derivative);
                telemetry1.addData("P", PID.gains.Kp);
                telemetry1.addData("I", PID.gains.Ki);
                telemetry1.addData("D", PID.gains.Kd);
            }
            if (currentMode == mode.ZieglerNichols) {
                /// PID.setPidForTuning(ultimateGain * 0.6f, (1.2f * ultimateGain) / oscillationPeriod, (3 * ultimateGain * oscillationPeriod) / 40);
                PID.gains.Reconstruct(ultimateGain * 0.45f, (0.54f * ultimateGain) / oscillationPeriod, 0);
            }
            PID.Loop();
            PID.getDashboardTelemetry().update();
        }
    }
}
