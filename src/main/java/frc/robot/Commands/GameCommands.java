package frc.robot.Commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public class GameCommands 
{
    public final Command StowAll() 
    { 
        return CommandUtils.withName("ShowAll", Commands.parallel(r.elevator.RaiseToStowed(), r.arm.PivotToStowed())); 
    }
    public final Command ScoreNet() 
    { 
        return CommandUtils.withName("ScoreNet", r.elevator.RaiseToNet()
            .andThen(r.arm.PivotToNet())
            .andThen(r.arm.ScoreAlgaeCommand().withTimeout(3)));
    }

    public final Command IntakeHigherAlgae()
    {
        return CommandUtils.withName("IntakeHigherAlgae", 
            r.arm.PivotTo180()
            .andThen(Commands.deadline(r.arm.IntakeAlgae(), r.arm.PivotToHigherAlgae()).withTimeout(6)))
            .andThen(r.arm.PivotTo180());
    }

    public final Command IntakeLowerAlgae() 
    {
        return CommandUtils.withName("IntakeLowerAlgae", Commands
            .parallel(r.elevator.RaiseToLowerAlgae(), r.arm.PivotToStowed())
            .andThen(Commands.deadline( r.arm.IntakeAlgae(), r.arm.PivotToLowerAlgae()).withTimeout(4)));
    }

    public final Command IntakeAlgae()
    {
        return CommandUtils.withName("IntakeAlgae", 
            r.arm.IntakeAlgae()
            .withTimeout(3));
    }

    public final Command IntakeFromCS() 
    {
        return CommandUtils.withName("IntakeFromCS", Commands
            .parallel(r.elevator.RaiseToCSIntake(), r.arm.PivotToStowed())
            .andThen(r.arm.IntakeCoral().withTimeout(4)));
    }

    public final Command ScoreCoralL3() 
    {
        return CommandUtils.withName("ScoreCoralL3", 
            r.arm.makePivotCommand(Rotation2d.fromDegrees(180))
            .andThen(r.arm.ScoreCoralCommand()));
    }

    public final Command ScoreCoraL4() 
    {
        return CommandUtils.withName("ScoreCoralL4", 
            Commands.parallel(r.elevator.RaiseToL4(), r.arm.PivotTo180())
            .andThen(r.arm.ScoreCoralCommand()));
    }

    private final Robot r;

    public GameCommands(Robot r)
    {
        this.r = r;
    }    
}
