package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ReefWaypointGenerator 
{
    public static class ReefEdgeWaypoints {
        public Pose2d AlgaeHigh;
        public Pose2d AlgaeLow;
        public Pose2d LeftL3Coral;
        public Pose2d LeftL4Coral;
        public Pose2d RightL3Coral;
        public Pose2d RightL4Coral;
    }

    private static final double HEX_RADIUS = 0.90;
    private static final double HEX_CENTER_X = 4.483;
    private static final double HEX_CENTER_Y = 4.030;
    private static final double SHIFT_AWAY_FROM_WALL = 0.425;

    private static Rotation2d ARM_SAG_ROTATION = Rotation2d.fromDegrees(0);

    private static Pose2d transformPoint(double centerX, double centerY, double faceAngle, double dx, double dy, Rotation2d relativeAngle) {
        // Convert the relative angle to an absolute angle by adding it to the face angle
        double absoluteAngle = faceAngle;

        double cosA = Math.cos(absoluteAngle);
        double sinA = Math.sin(absoluteAngle);

        double localX = centerX + cosA * dx - sinA * dy;
        double localY = centerY + sinA * dx + cosA * dy;

        return new Pose2d(localX, localY, Rotation2d.fromRadians(faceAngle).plus(relativeAngle));
    }

    public static List<ReefEdgeWaypoints> generateHexagonPoses() 
    {
        List<ReefEdgeWaypoints> poses = new ArrayList<>();

        for (int i = 0; i < 6; i++) 
        {
            // The angle for each face is calculated as 60 * i
            double angle = -Math.toRadians(60 * i);
            angle += Math.PI;

            double edgeCenterX = HEX_CENTER_X + HEX_RADIUS * Math.cos(angle);
            double edgeCenterY = HEX_CENTER_Y + HEX_RADIUS * Math.sin(angle);

            ReefEdgeWaypoints waypoints = new ReefEdgeWaypoints();

            waypoints.AlgaeHigh = FieldConstants.FlipIfRequired(transformPoint(edgeCenterX, edgeCenterY, angle, SHIFT_AWAY_FROM_WALL - 0.127 + 0.072, 0.17 + 0.127 + 0.0381, Rotation2d.fromDegrees(-90).minus(ARM_SAG_ROTATION)), Alliance.Blue);
            waypoints.AlgaeLow = FieldConstants.FlipIfRequired(transformPoint(edgeCenterX, edgeCenterY, angle, SHIFT_AWAY_FROM_WALL - 0.127 + 0.072, -0.15 + 0.0127, Rotation2d.fromDegrees(90).minus(ARM_SAG_ROTATION)), Alliance.Blue);
            
            waypoints.LeftL4Coral = FieldConstants.FlipIfRequired(transformPoint(edgeCenterX, edgeCenterY, angle, SHIFT_AWAY_FROM_WALL + 0.0508 + 0.28 - 0.0508, -0.20 - 0.0508 + 0.0125 + 0.0381 - 0.0254, Rotation2d.fromDegrees(180).minus(ARM_SAG_ROTATION)), Alliance.Blue);
            waypoints.RightL4Coral = FieldConstants.FlipIfRequired(transformPoint(edgeCenterX, edgeCenterY, angle, SHIFT_AWAY_FROM_WALL + 0.0508 + 0.28 - 0.0508, -0.20 + 0.3302 - 0.0762 + 0.00625 + 0.0127, Rotation2d.fromDegrees(180).minus(ARM_SAG_ROTATION)), Alliance.Blue);
            
            waypoints.RightL3Coral = FieldConstants.FlipIfRequired(transformPoint(edgeCenterX, edgeCenterY, angle, SHIFT_AWAY_FROM_WALL + 0.19 - 0.0508, 0.07 - 0.1016 + 0.0762 - 0.0254 + 0.03, Rotation2d.fromDegrees(180).minus(ARM_SAG_ROTATION)), Alliance.Blue);
            waypoints.LeftL3Coral = FieldConstants.FlipIfRequired(transformPoint(edgeCenterX, edgeCenterY, angle, SHIFT_AWAY_FROM_WALL + 0.19 - 0.0508, 0.07 - 0.3302 - 0.1016 + 0.0762, Rotation2d.fromDegrees(180).minus(ARM_SAG_ROTATION)), Alliance.Blue);

            // Add the generated waypoints to the list
            poses.add(waypoints);
        }
    

        return poses;
    }

    public static void printWaypoints(List<ReefEdgeWaypoints> waypoints) {
        for (int i = 0; i < waypoints.size(); i++) {
            ReefEdgeWaypoints wp = waypoints.get(i);
            System.out.println("Face " + i + " Waypoints:");  // Start count from 0
            System.out.printf("  AlgaeHigh: (%.3f, %.3f, %.2f°)\n", wp.AlgaeHigh.getX(), wp.AlgaeHigh.getY(), wp.AlgaeHigh.getRotation().getDegrees());
            System.out.printf("  AlgaeLow: (%.3f, %.3f, %.2f°)\n", wp.AlgaeLow.getX(), wp.AlgaeLow.getY(), wp.AlgaeLow.getRotation().getDegrees());
            System.out.printf("  LeftL4Coral: (%.3f, %.3f, %.2f°)\n", wp.LeftL4Coral.getX(), wp.LeftL4Coral.getY(), wp.LeftL4Coral.getRotation().getDegrees());
            System.out.printf("  RightL4Coral: (%.3f, %.3f, %.2f°)\n", wp.RightL4Coral.getX(), wp.RightL4Coral.getY(), wp.RightL4Coral.getRotation().getDegrees());
            System.out.printf("  LeftL3Coral: (%.3f, %.3f, %.2f°)\n", wp.LeftL3Coral.getX(), wp.LeftL3Coral.getY(), wp.LeftL3Coral.getRotation().getDegrees());
            System.out.printf("  RightL3Coral: (%.3f, %.3f, %.2f°)\n", wp.RightL3Coral.getX(), wp.RightL3Coral.getY(), wp.RightL3Coral.getRotation().getDegrees());
            System.out.println();
        }
    }

    public static void main(String[] args) {
        List<ReefEdgeWaypoints> waypoints = generateHexagonPoses();
        printWaypoints(waypoints);
    }
}