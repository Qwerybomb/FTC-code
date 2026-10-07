package org.firstinspires.ftc.teamcode.Modules;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class PidController {

    // Pre-Calculated Gains //
    private float Kp;
    private float Ki;
    private float Kd;
    private float Dt;

    // Motor //
    private DcMotor Motor;

    // Error visualization helper //
    Telemetry telemetry;

    // Error & Error calculatory values //
    private float Target;
    private final ArrayList<Float> Error = new ArrayList<Float>();

    /**
     * Assign initial values for all robot or externally dependent objects
     * @param HwMap Hardware map of the robot, from which to retrieve the motor
     * @param telemetry Telemetry for the Driver Station
     * @param Target Desired Initial Target value
     */
    public PidController(HardwareMap HwMap, Telemetry telemetry, Float Target) {
        this.telemetry = telemetry;
        UpdateTarget(Target);
        Motor = HwMap.get(DcMotor.class, "ExampleMotor");
    }

    /**
     * Primary control loop for the PID mechanism.
     */
    public void PidLoop() {
        Error.add(Target - CalculateProcessVariable());
        Motor.setPower(CalculateProportional() + CalculateDerivative() + CalculateIntegral());
        try {
            Thread.sleep((long) Dt);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * function to calculate the P in PID
     * @return the P in PID
     */
    private float CalculateProportional() {
        return Kp * Error.get(Error.size() - 1);
    }

    /**
     * function to calculate the I in PID
     * @return the I in PID
     */
    private float CalculateIntegral() {
        return Ki;
    }

    /**
     * function to calculate the D in PID
     * @return the D in PID
     */
    private float CalculateDerivative() {
        return Kd * ((Error.get(Error.size() - 1) - Error.get(Error.size() - 2)) / Dt);
    }

    /**
     * Calculate the Process Variable.
     * @return Process Variable
     */
    private float CalculateProcessVariable() {
        return Motor.getCurrentPosition();
    }

    /**
     * Change the current target into a new Target.
     * @param Target The new Target
     */
    public void UpdateTarget(Float Target) {
        this.Target = Target;
    }
}
