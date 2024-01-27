package frc.lib;

public abstract class Util {
    public static boolean epislonEquals(double x, double y, double epislon) {
        return Math.abs(x - y) < epislon;
    }

    public static boolean epislonEquals(double x, double y) {
        return epislonEquals(x, y, 1E-6);
    }
}
