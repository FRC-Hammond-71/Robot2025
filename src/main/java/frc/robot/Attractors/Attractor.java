package frc.robot.Attractors;

import edu.wpi.first.math.geometry.Translation2d;

public interface Attractor {

    double getMagnitude(Translation2d rPose);

    Translation2d getPosition(Translation2d rPose);
    
    public static boolean isInRange(Translation2d rPose, Attractor attractor)
    {
        return attractor.getMagnitude(rPose) > 0;
    }

    /**
     * Finds and returns the nearest attractor from a list of attractors based on magnitude.
     *
     * @param rPose The Robot reference position used to calculate the magnitude.
     * @param attractors A collection of attractors to search through.
     * @param <T> A type that extends Attractor, ensuring it has a getMagnitude method.
     * @return The attractor with the highest magnitude relative to rPose, or null if no valid attractor is found.
     */
    public static <T extends Attractor> T getNearestAttractor(Translation2d rPose, T[] attractors)
    {
        T nearestAttractor = null;
        // Save the nearest attractor mag to reduce re-doing magnitude calculations.
        double nearestAttractorMag = 0;

        for (T attractor : attractors)
        {
            double attractorMag = attractor.getMagnitude(rPose);

            if (nearestAttractor == null || attractorMag > nearestAttractorMag)
            {
                nearestAttractor = attractor;
                nearestAttractorMag = attractorMag;
            }
        } 

        // Because we assign nearestAttractor at first without checking others, check if the mag is zero.
        return (nearestAttractor != null && nearestAttractorMag != 0) ? nearestAttractor : null;
    }
}
