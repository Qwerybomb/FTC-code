package org.firstinspires.ftc.teamcode.Modules;

public class Gains {
    public float Kp = 0;
    public float Ki = 0;
    public float Kd = 0;

    /**
     * Constructor for gains. Internally calls Reconstruct()
     */
    public Gains(float Kp, float Ki, float Kd) {
        Reconstruct(Kp,Ki,Kd);
    }

    /**
     * set all gains at once.
     * @param Kp Proportional Gain
     * @param Ki Integral Gain
     * @param Kd Derivative Gain
     */
    public void Reconstruct(float Kp, float Ki, float Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
}
