package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ReefWaypointGenerator.ReefEdgeWaypoints;
import frc.robot.Attractors.Attractor;
import frc.robot.Attractors.LineAttractor;
import frc.robot.Attractors.PointAttractor;
import frc.robot.Attractors.Controllers.FaceController;
import frc.robot.Attractors.Controllers.ReefPoseController;
import frc.robot.Commands.CommandUtils;
import frc.robot.Utilities.ChassisSpeedsUtils;

public class DriverAssistance
{
    /** Encapsulates driver assist settings */
    private static class DriverAssistConfig {
        final int reefCoralLevel;
        final int reefCoralSide;
        final int reefAlgaePosition;
        final int reefFace;

        DriverAssistConfig(int coralLevel, int coralSide, int algaePosition, int face) {
            this.reefCoralLevel = coralLevel;
            this.reefCoralSide = coralSide;
            this.reefAlgaePosition = algaePosition;
            this.reefFace = face;
        }
    }

    public static class DriverAssistOutput {
        public final ChassisSpeeds speeds;
        public final Command recommendedCommand;

        public DriverAssistOutput(ChassisSpeeds speeds, Command recommendedCommand) {
            this.speeds = speeds;
            this.recommendedCommand = recommendedCommand;
        }
    }

    private final ReefPoseController[] reefEdgeAttractors;
    public final FaceController[] coralStationAttractors;
    public final FaceController netAttractor;
    public final FaceController processorAttractor;

    private Robot r;

    public DriverAssistance(Robot r)
    {
        this.r = r;

        final var generatedReefWaypoints = ReefWaypointGenerator.generateHexagonPoses();
        this.reefEdgeAttractors = new ReefPoseController[generatedReefWaypoints.size()];
        ReefWaypointGenerator.printWaypoints(generatedReefWaypoints);

        double reefMinDistanceInMeters = 1.5;
        for (int i = 0; i < generatedReefWaypoints.size(); i++)
        {
            ReefEdgeWaypoints waypoints = generatedReefWaypoints.get(i);
            this.reefEdgeAttractors[i] = new ReefPoseController(

                new FaceController(waypoints.AlgaeHigh.getRotation(), new PointAttractor(waypoints.AlgaeHigh.getTranslation(), reefMinDistanceInMeters)),
                new FaceController(waypoints.AlgaeLow.getRotation(), new PointAttractor(waypoints.AlgaeLow.getTranslation(), reefMinDistanceInMeters)),
                
                new FaceController(waypoints.LeftL3Coral.getRotation(), new PointAttractor(waypoints.LeftL3Coral.getTranslation(), reefMinDistanceInMeters)),
                new FaceController(waypoints.RightL3Coral.getRotation(), new PointAttractor(waypoints.RightL3Coral.getTranslation(), reefMinDistanceInMeters)),
                
                new FaceController(waypoints.LeftL4Coral.getRotation(), new PointAttractor(waypoints.LeftL4Coral.getTranslation(), reefMinDistanceInMeters)),
                new FaceController(waypoints.RightL4Coral.getRotation(), new PointAttractor(waypoints.RightL4Coral.getTranslation(), reefMinDistanceInMeters))

            );
        }

        this.r.swerve.m_field.getObject("Net Scoring").setPoses(
            new Pose2d(FieldConstants.AlgaeNetLine().TranslationA, FieldConstants.AlgaeNetRotation()),
            new Pose2d(FieldConstants.AlgaeNetLine().TranslationB, FieldConstants.AlgaeNetRotation())
        );

        Pose2d coralStationLeft = FieldConstants.CoralStationLeftLineup();
        Pose2d coralStationRight = FieldConstants.CoralStationRightLineup();
        this.coralStationAttractors = new FaceController[] {
            new FaceController(coralStationLeft.getRotation(), new PointAttractor(coralStationLeft.getTranslation(), 1)),
            new FaceController(coralStationRight.getRotation(), new PointAttractor(coralStationRight.getTranslation(), 1))
        };

        this.netAttractor = new FaceController(FieldConstants.AlgaeNetRotation(), new LineAttractor(FieldConstants.AlgaeNetLine(), 1));

        this.processorAttractor = new FaceController(FieldConstants.CoralProcessor().getRotation(), new PointAttractor(FieldConstants.CoralProcessor().getTranslation(), 1));
    }

    /** Reads SmartDashboard values into a structured object */
    private DriverAssistConfig getDriverAssistConfig() 
    {
        return new DriverAssistConfig(
            (int) SmartDashboard.getNumber("DriverAssisted/ReefCoralLevel", -1),
            (int) SmartDashboard.getNumber("DriverAssisted/ReefCoralSide", -1),
            (int) SmartDashboard.getNumber("DriverAssisted/AlgaePosition", -1),
            (int) SmartDashboard.getNumber("DriverAssisted/ReefFace", -1)
        );
    }

    public DriverAssistOutput assistDriver(BooleanSupplier onlyWhilePredicate) 
    {
        Pose2d rPose = this.r.swerve.getPose();
        DriverAssistConfig config = getDriverAssistConfig();

        boolean doAssistInsertCoral = config.reefCoralLevel > 0 && config.reefCoralSide >= 0 && config.reefFace >= 0;
        boolean doAssistIntakeAlgaeFromReef = config.reefAlgaePosition >= 0 && config.reefFace >= 0;

        if (doAssistInsertCoral || doAssistIntakeAlgaeFromReef) {
            DriverAssistOutput reefAssist = assistReef(config, rPose, onlyWhilePredicate);
            if (!ChassisSpeedsUtils.isEmpty(reefAssist.speeds)) {
                return reefAssist;
            }
        }

        DriverAssistOutput coralStationAssist = assistCoralStation(rPose, onlyWhilePredicate);
        if (!ChassisSpeedsUtils.isEmpty(coralStationAssist.speeds)) {
            return coralStationAssist;
        }

        DriverAssistOutput netAssist = assistNet(rPose, onlyWhilePredicate);
        if (!ChassisSpeedsUtils.isEmpty(netAssist.speeds)) {
            return netAssist;
        }

        DriverAssistOutput processorAssist = assistProcessor(rPose, onlyWhilePredicate);
        if (!ChassisSpeedsUtils.isEmpty(processorAssist.speeds)) {
            return processorAssist;
        }

        return new DriverAssistOutput(new ChassisSpeeds(), null); // No assistance needed
    }

    private DriverAssistOutput assistCoralStation(Pose2d rPose, BooleanSupplier onlyWhilePredicate) 
    {
        FaceController nearestController = Attractor.getNearestAttractor(rPose.getTranslation(), this.coralStationAttractors);

        if (nearestController != null && Attractor.isInRange(rPose.getTranslation(), nearestController)) {
            ChassisSpeeds speeds = nearestController.calculate(rPose);
            
            // if (nearestController.getDistanceInMeters(rPose.getTranslation()) <= 0.1016 && nearestController.isAtRotation()) {
            //     return new DriverAssistOutput(speeds, this.r.gameCommands.IntakeFromCS());
            // }

            return new DriverAssistOutput(speeds, this.r.gameCommands.IntakeFromCS()
                .onlyWhile(onlyWhilePredicate)
                .finallyDo(() -> this.r.elevator.RaiseToStowed()));
        }

        return new DriverAssistOutput(new ChassisSpeeds(), null);
    }

    private DriverAssistOutput assistNet(Pose2d rPose, BooleanSupplier onlyWhilePredicate)
    {
        return new DriverAssistOutput(
            this.netAttractor.calculate(rPose),
            this.r.gameCommands.ScoreNet()
                .onlyWhile(onlyWhilePredicate)
                .finallyDo(() -> {
                    this.r.arm.turnToStowed();
                    this.r.elevator.setPositions(0);
                }));
    }

    private DriverAssistOutput assistProcessor(Pose2d rPose, BooleanSupplier onlyWhilePredicate)
    {
        return new DriverAssistOutput(
            this.processorAttractor.calculate(rPose),
            // (this.processorAttractor.getDistanceInMeters(rPose.getTranslation()) <= 0.1016 && this.processorAttractor.isAtRotation()) ? this.r.gameCommands.ScoreProcessor() : null);
            this.r.gameCommands.ScoreProcessor().onlyWhile(onlyWhilePredicate));
    }

    /** Assists the driver in scoring coral or intaking algae */
    private DriverAssistOutput assistReef(DriverAssistConfig config, Pose2d rPose, BooleanSupplier onlyWhilePredicate) {
        FaceController targetAttractor = getReefTargetAttractor(config);

        if (targetAttractor != null)
        {
            this.r.swerve.m_field.getObject("nearestAttractor").setPose(new Pose2d(targetAttractor.getPosition(rPose), Rotation2d.fromDegrees(0)));
            
            return new DriverAssistOutput(
                targetAttractor.calculate(rPose),
                getReefTargetCommand(config, onlyWhilePredicate)
            );
        }
        return new DriverAssistOutput(
            new ChassisSpeeds(),
            getReefTargetCommand(config, onlyWhilePredicate)
        );
    }

    /** Determines the correct attractor for reef operations */
    private FaceController getReefTargetAttractor(DriverAssistConfig config) {
        if (config.reefFace == -1) 
        {
            return null; // Invalid face index
        }
        ReefPoseController reefController = this.reefEdgeAttractors[config.reefFace];

        if (config.reefAlgaePosition == 0)
        {
            return reefController.AlgaeLowController;
        }
        else if (config.reefAlgaePosition == 1)
        {
            return reefController.AlgaeHighController;
        }

        return switch (config.reefCoralLevel) {
            case 3 -> (config.reefCoralSide == 0) ? reefController.LeftCoralL3Controller : reefController.RightCoralL3Controller;
            case 4 -> (config.reefCoralSide == 0) ? reefController.LeftCoralL4Controller : reefController.RightCoralL4Controller;
            default -> null;
        };
    }

    private Command getReefTargetCommand(DriverAssistConfig config, BooleanSupplier onlyWhilePredicate) {
        if (config.reefFace < 0 || config.reefFace >= this.reefEdgeAttractors.length) {
            return null; // Invalid face index
        }

        if (config.reefAlgaePosition == 0)
        {
            return this.r.gameCommands
                .IntakeLowerAlgae()
                .onlyWhile(onlyWhilePredicate)
                .finallyDo(() -> r.arm.PivotToStowed());
        }
        else if (config.reefAlgaePosition == 1)
        {
            return CommandUtils.withName("IntakeHigherAlgae", r.arm.PivotTo180()
                .andThen(Commands.parallel(
                    r.launcher.cmdIntakeAlgae(),
                    r.arm.PivotToHigherAlgae()
                    ))
                .onlyWhile(onlyWhilePredicate)
                .finallyDo(() -> {
                    // r.elevator.setPositions(r.elevator.getHeight() + 4);
                    r.arm.setTargetRotation(r.arm.k180Angle);
                }));
        }

        switch (config.reefCoralLevel) {
            case 3:
                return this.r.gameCommands.ScoreCoralL3();
            case 4:
                return this.r.gameCommands.ScoreCoraL4();
            default:
                return null;
        }
    }
}
