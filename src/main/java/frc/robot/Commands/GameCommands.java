package frc.robot.Commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
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
            .andThen(r.arm.ScoreAlgaeCommand()))
            .andThen(this.StowAll());
    }
    public final Command IntakeHigherAlgae()
    {
        return CommandUtils.withName("IntakeHigherAlgae", Commands.parallel(r.arm.PivotToHigherAlgae(), r.arm.IntakeAlgae().withTimeout(3)));
    }
    public final Command IntakeLowerAlgae() 
    {
        return CommandUtils.withName("IntakeLowerAlgae", Commands
            .parallel(r.elevator.RaiseToLowerAlgae(), r.arm.PivotToStowed())
            .andThen(Commands.parallel(r.arm.PivotToLowerAlgae(), r.arm.IntakeAlgae().withTimeout(3))));
    }
    public final Command IntakeFromCS() 
    {
        return CommandUtils.withName("IntakeFromCS", Commands
            .parallel(r.elevator.RaiseToCSIntake(), r.arm.PivotToStowed())
            .andThen(r.arm.IntakeCoral().withTimeout(3))
            .andThen(this.StowAll()));
    }
    public final Command ScoreCoralL3() 
    {
        return CommandUtils.withName("ScoreCoralL3", 
            r.arm.makePivotCommand(Rotation2d.fromDegrees(35))
            .andThen(r.arm.ScoreCoralCommand()));
    }

    public final Command ScoreCoral4() 
    {
        return CommandUtils.withName("ScoreCoralL4", 
            Commands.parallel(r.elevator.RaiseToL4(), r.arm.PivotTo180()));
    }

    private final Robot r;

    public GameCommands(Robot r)
    {
        this.r = r;
    }    
}
