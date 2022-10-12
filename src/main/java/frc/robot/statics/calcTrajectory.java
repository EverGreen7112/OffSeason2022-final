package frc.robot.statics;
public class calcTrajectory {

    public static final double G = 9.807;

    /**Calculates the speed needed to reach the target given an angle.
     * @param distance The distance from the target.
     * @param startHeight The height of the starting position.
     * @param angle The shooting angle.
     * @param VerticalError This value increases the target height.
     * @param HorizontalError This value increases the target distance.
     * @param targetHeight The height of the target.
     * 
     * @return The shooting speed in double.
     */

    public static double calcSpeed(double distance, double startHeight, double angle, double VerticalError, double HorizontalError, double targetHeight) {
        double sinA = Math.sin(Math.toRadians(angle));
        double cosA = Math.cos(Math.toRadians(angle));

        double a = ((G/2) * Math.pow(HorizontalError + distance, 2)) / Math.pow(cosA, 2);

        double b = (((HorizontalError + distance) * sinA) / cosA) - (targetHeight + VerticalError - startHeight);

        return Math.sqrt(a / b);
    }

    /**Calculates the speed needed to reach the target given an angle.
     * Deafult target height is 2.64.
     * @param distance The distance from the target.
     * @param startHeight The height of the starting position.
     * @param angle The shooting angle.
     * @param VerticalError This value increases the target height.
     * @param HorizontalError This value increases the target distance.
     * 
     * @return The shooting speed in double.
     */

    public static double calcSpeed(double distance, double startHeight, double angle, double VerticalError, double HorizontalError) {
        return calcSpeed(distance, startHeight, angle, VerticalError, HorizontalError, 2.64);
    }

    /**Calculates the speed needed to reach the target given an angle.
     * Deafult target height is 2.64 with intentional vertical error of 0.15 and horizontal error of 0.15.
     * @param distance The distance from the target.
     * @param startHeight The height of the starting position.
     * @param angle The shooting angle.
     * 
     * @return The shooting speed in double.
     */

    public static double calcSpeed(double distance, double startHeight, double angle) {
        return calcSpeed(distance, startHeight, angle, 0.15, 0.15, 2.64);
    }

    public static double calcHorizontalDistance(double z, double y, double h) {
        double w = Math.sqrt(Math.pow(z, 2) + Math.pow(y, 2));

        double d = Math.sqrt(Math.pow(w, 2) + Math.pow(h, 2));
        
        return d;
    }
}
