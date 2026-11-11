package org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class PidController {

    // Pre-Calculated Gains //
    public Gains gains = new Gains(0,0,0);

    // Timings (all in ms) //
    private float activeDeltaTime = 0;
    private float previousTimePoint = 0;
    private float sampleTimingPrevious = 0;
    private int sampleTime = 50;
    private ElapsedTime time;

    // integration //
    private float integral = 0;

    // Error & Error calculatory values //
    public float Target;
    private float currentError;
    private float previousError;

    // calculated new error value //
    protected float my_Ut = 0;

    // telemetry //
    private final Telemetry telemetry;

    public PidController(Float Target, int sampleSpeed) {
       updateTarget(Target);
       setMySampleSpeed(sampleSpeed);

        time = new ElapsedTime(0);
        time.reset();

        this.telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    /**
     * Primary control loop for the PID mechanism.
     * The telemetry for the dashboard is updated here. for custom telemetry additions,
     * please add them before calling Loop();
     */
    public void Loop() {

        if (sampleTimingPrevious + sampleTime > time.milliseconds()) {
            return;
        }
        sampleTimingPrevious = (float) time.milliseconds();

        // calculate the active delta time.
        activeDeltaTime = (float) (time.milliseconds() - previousTimePoint);
        previousTimePoint = (float) time.milliseconds();

        // set the error last iteration to the error this iteration
        previousError = currentError;

        // set the current error to the target - PV
        currentError = Target - CalculateProcessVariable(activeDeltaTime);

        my_Ut = CalculateProportional() + CalculateDerivative() + CalculateIntegral();

        addEssentialTelemetry();

        customLoopControls();
    }

    /**
     * Custom loop logic. Called once at the end of Loop(). remember to call Loop() instead of this function in all final implementations.
     */
    protected abstract void customLoopControls();

    /**
     * function to calculate the P in PID
     * @return the P in PID
     */
    private float CalculateProportional() {
        return gains.Kp * currentError;
    }

    /**
     * function to calculate the I in PID. Reminder -> All time in Ms
     * @return the I in PID
     */
    private float CalculateIntegral() {
        integral = gains.Ki * ((integral + currentError) * activeDeltaTime);
        return integral;
    }

    /**
     * function to calculate the D in PID
     * @return the D in PID
     */
    private float CalculateDerivative() {
        return gains.Kd * (currentError - previousError) / activeDeltaTime;
    }

    /**
     * Calculate the Process Variable (RPM). This PV will be used in the pre-built Loop().
     * @param deltaTime the time the previous iteration took to process in milliseconds.
     * @return Process Variable
     */
    protected abstract float CalculateProcessVariable(float deltaTime);

    /**
     * remaps values from one range A to range B. example, a value of 500 in a 0-1000 scale can be remapped to 0.5 on a 0-1 scale.
     * @param low1 range A lower expected
     * @param high1 range A higher expected
     * @param low2 range B lower expected
     * @param high2 range B higher expected
     * @param value value to remap.
     * @return remapped value
     */
    protected float remap(float low1, float high1, float low2, float high2, float value) {
        return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
    }

    /**
     * Change the current target into a new Target.
     * @param Target The new Target
     */
    public void updateTarget(Float Target) {
        this.Target = Target;
    }

    /**
     * retrieve the final calculated U(t) from the PID.
     * @return final calculated U(t) from the PID.
     */
    public float getCalculatedUt() {
        return my_Ut;
    }

    /**
     * Set the sample speed in MILLISECONDS (ms). will sample every set of this Milliseconds.
     * @param sampleTime new sample time in Milliseconds (ms)
     */
    public void setMySampleSpeed(int sampleTime) {
        this.sampleTime = sampleTime;
    }

    private void addEssentialTelemetry() {
        telemetry.addData("bounds_bottom", 0);
        telemetry.addData("target", Target);
        telemetry.addData("currentError", currentError);
        telemetry.addData("Process Variable (RPM)", Target - currentError);
        telemetry.addData("current delta time", activeDeltaTime);
        telemetry.update();
    }

    public Telemetry getDashboardTelemetry() {
        return telemetry;
    }
}
