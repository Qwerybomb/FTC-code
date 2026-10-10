package org.firstinspires.ftc.teamcode.Modules;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class PidController {

    // Pre-Calculated Gains //
    private final float Kp = 0;
    private final float Ki = 0;
    private final float Kd = 0;

    // pre determined loop time in Ms //
    private final float Dt = 100;

    // Motor //
    private DcMotor Motor;
    private float ECPerRotation = 100;

    // elapsed time //
    private ElapsedTime elapsedTime;

    // integration //
    private float integral = 0;
    private float integrationStartTime = 0;

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
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
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
     * function to calculate the I in PID. Reminder -> All time in MS
     *
     * @return the I in PID
     */
    private double CalculateIntegral() {
        float startTime = integrationStartTime;
        float endTime = (float) elapsedTime.milliseconds();
        float subdivisions = 50;

        float h = (endTime - startTime) / subdivisions;
        float sum = 0.5f * (getErrorAtTime(startTime) + getErrorAtTime(endTime));
        for (int i = 1; i < subdivisions; i++) {
            sum += getErrorAtTime(startTime + i * h);
        }


        integrationStartTime = (float) elapsedTime.milliseconds();
        integral += sum * h;
        return integral * Ki;
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
     * function to calculate error at the given time. uses linear easing between values.
     * @param Time time to request in MS
     * @return value at the time.
     */
    private float getErrorAtTime(float Time) {
        int errorPoints = Error.size() - 1;
        float currentRunTime = (float) elapsedTime.milliseconds();

        float currentPosition = remap(0, currentRunTime, 0, errorPoints, Time);

        int errorBottomBound = (int) Math.floor(currentPosition);
        int errorTopBound = (int) Math.ceil(currentPosition);
        float deviation = errorTopBound - currentPosition;

        return (Error.get(errorBottomBound) * (deviation)) + (Error.get(errorTopBound) * (1 - deviation));
    }

    private float remap(float low1, float high1, float low2, float high2, float value) {
        return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
    }

    /**
     * Change the current target into a new Target.
     * @param Target The new Target
     */
    public void UpdateTarget(Float Target) {
        this.Target = Target;
    }
}
