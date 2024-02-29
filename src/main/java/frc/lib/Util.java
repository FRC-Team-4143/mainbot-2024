package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class Util {
    public static boolean epislonEquals(double x, double y, double epislon) {
        return Math.abs(x - y) < epislon;
    }

    public static boolean epislonEquals(double x, double y) {
        return epislonEquals(x, y, 1E-6);
    }

    public static boolean epislonEquals(Rotation2d x, Rotation2d y, double epislon) {
        return Math.abs(x.minus(y).getRadians()) < epislon;
    }
}
