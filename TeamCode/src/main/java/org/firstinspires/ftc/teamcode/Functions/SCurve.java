package org.firstinspires.ftc.teamcode.Functions;

public class SCurve {
    public static double Stabilize(double new_accel, double current_accel, double MAX_ACCELERATION_DEVIATION) {
        double dev = new_accel - current_accel;
        return Math.abs(dev) > MAX_ACCELERATION_DEVIATION ? current_accel + MAX_ACCELERATION_DEVIATION * dev / Math.abs(dev) : new_accel;
    }
}
