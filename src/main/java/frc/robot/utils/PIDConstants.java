package frc.robot.utils;

public class PIDConstants {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public PIDConstants(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

}