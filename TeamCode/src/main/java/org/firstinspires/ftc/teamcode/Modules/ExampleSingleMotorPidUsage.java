package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PID.Gains;
import org.firstinspires.ftc.teamcode.PID.PidController;

public class ExampleSingleMotorPidUsage extends LinearOpMode {

    // potential targets
    public enum targets {
        Lowest(0),
        Highest(300);

        final float RPM;

        targets(float RPM) {
            this.RPM = RPM;
        }
    }


    PidController pidController;
    Gains customGains;

    @Override
    public void runOpMode() throws InterruptedException {

        // construct the pidController as a PidSingleMotor.
        pidController = new PidSingleMotor(hardwareMap, 200F, 50);

        // simple gains object that applies a 0.2 proportional gain
        customGains = new Gains(0.2f,0,0);

        // apply our custom gains to the pidController.
        pidController.gains = customGains;

        waitForStart();

        while (opModeIsActive()) {
            pidController.Loop();

            // make the target value change depending on the bumper pressed.
            if (gamepad1.left_bumper) {
                pidController.updateTarget(targets.Highest.RPM);
            }
            if (gamepad1.right_bumper) {
                pidController.updateTarget(targets.Lowest.RPM);
            }

            // take samples at longer intervals when we are closer to the target (200).
            if (pidController.getCalculatedUt() > 100) {
                pidController.setMySampleSpeed(100);
            } else {
                pidController.setMySampleSpeed(50);
            }
        }
    }
}
