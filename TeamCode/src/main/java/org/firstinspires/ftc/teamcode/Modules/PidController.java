package org.firstinspires.ftc.teamcode.Modules;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PidController {

    // Pre-Calculated Gains //
    private float Kp = 0;
    private float Ki = 0;
    private float Kd = 0;

    // Timings (all in ms) //
    private final float Dt = 100;

    // Motor //
    private final DcMotor Motor;
    private final float encodersPerRevolution = 537.7f;
    private final float maxRPM = 312;

    // integration //
    private float integral = 0;

    // Error visualization helper //
    Telemetry telemetry;
    boolean printTelemetry;

    // Error & Error calculatory values //
    private float Target;
    private float currentError;
    private float previousError;

    // process value calculations //
    private float motorPreviousEncoderCount = 0;

    /**
     * Assign initial values for all robot or externally dependent objects
     * @param HwMap Hardware map of the robot, from which to retrieve the motor
     * @param telemetry Telemetry for the Driver Station
     * @param Target Desired Initial Target value
     */
    public PidController(HardwareMap HwMap, Float Target, boolean printTelemetry) {
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
        this.printTelemetry = printTelemetry;

        UpdateTarget(Target);

        Motor = HwMap.get(DcMotor.class, "PidMotor");
        Motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Primary control loop for the PID mechanism.
     */
    public void Loop() {
        previousError = currentError;
        currentError = Target - CalculateProcessVariable();
        currentError = remap(0, maxRPM, 0, 1, currentError);
        Motor.setPower(CalculateProportional() + CalculateDerivative() + CalculateIntegral());

        if (printTelemetry) {
            telemetry.addData("currentError", currentError);
            telemetry.addData("Process Variable (RPM)", Target - remap(0, 1, 0, maxRPM, currentError));
            telemetry.addData("Motor Power (u(t))", Motor.getPower());
        }

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
        return Kp * currentError;
    }

    /**
     * function to calculate the I in PID. Reminder -> All time in Ms
     * @return the I in PID
     */
    private float CalculateIntegral() {
        integral = Ki * ((integral + currentError) * Dt);
        return integral;
    }

    /**
     * function to calculate the D in PID
     * @return the D in PID
     */
    private float CalculateDerivative() {
        return Kd * (currentError - previousError) / Dt;
    }

    /**
     * Calculate the Process Variable (RPM) using the previously known encoder count and the amount of encoder counts per revolution, as well at the known time-per-iteration (could be improved with added deltaTime).
     * @return Process Variable
     */
    private float CalculateProcessVariable() {

        // calculate the process variable
        float processVariable =  ((Motor.getCurrentPosition() - motorPreviousEncoderCount) / encodersPerRevolution);
        processVariable *=  (60000 / Dt);

        // set previous values to be read next [function call] -> (iteration)
        motorPreviousEncoderCount = Motor.getCurrentPosition();
        return processVariable;
    }

    /**
     * remaps values from one range A to range B. example, a value of 500 in a 0-1000 scale can be remapped to 0.5 on a 0-1 scale.
     * @param low1 range A lower expected
     * @param high1 range A higher expected
     * @param low2 range B lower expected
     * @param high2 range B higher expected
     * @param value value to remap.
     * @return remapped value
     */
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

    /**
     * set the proportional gain of the PID controller
     * @param proportional proportional gain
     */
    public void setProportional(float proportional) {
        Kp = proportional;
    }

    /**
     * set the Integral gain of the PID controller
     * @param integral Integral gain
     */
    public void setIntegral(float integral) {
        Ki = integral;
    }

    /**
     * set the Derivative gain of the PID controller
     * @param derivative Derivative gain
     */
    public void setDerivative(float derivative) {
        Kd = derivative;
    }

    public void updateTelemetry() {
        telemetry.update();
    }
    public Telemetry getTelemetry() {
        return this.telemetry;
    }
}
