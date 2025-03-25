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
        return CommandUtils.withName("ScoreNet", r.elevator.RaiseToNet().alongWith(r.arm.PivotToNet())
            .andThen(r.launcher.cmdScoreAlgae().withTimeout(0.5)));
    }

    public final Command IntakeHigherAlgae()
    {
        return CommandUtils.withName("IntakeHigherAlgae", 
            r.arm.PivotTo180()
            .andThen(Commands.race(
                r.launcher.cmdIntakeAlgae(), 
                r.arm.PivotToHigherAlgae().withTimeout(2)
                    .andThen(r.elevator.makeRaiseToCommand(r.elevator.getHeight() + 1))
                    .andThen(r.arm.PivotTo180())
            )));
    }

    public final Command AutoIntakeLowerAlgae() 
    {
        return CommandUtils.withName("IntakeLowerAlgae", Commands
            .parallel(r.elevator.RaiseToLowerAlgae(), r.arm.PivotToStowed())
            .andThen(Commands.deadline(r.launcher.cmdIntakeAlgae().withTimeout(3), r.arm.PivotToLowerAlgae())));
    }
    public final Command IntakeLowerAlgae() 
    {
        return CommandUtils.withName("IntakeLowerAlgae", Commands
            .parallel(r.elevator.RaiseToLowerAlgae(), r.arm.PivotToStowed())
            .andThen(Commands.deadline(r.launcher.cmdIntakeAlgae(), r.arm.PivotToLowerAlgae())));
    }

    public final Command IntakeAlgae()
    {
        return CommandUtils.withName("IntakeAlgae", 
            r.launcher.cmdAutoIntakeAlgae().withTimeout(3));
    }

    public final Command ScoreProcessor()
    {
        return CommandUtils.withName("ScoreProcessor", Commands.parallel(r.arm.PivotToStowed(), r.elevator.RaiseToStowed()).andThen(r.launcher.cmdScoreAlgae()));
    }

    public final Command IntakeFromCS() 
    {
        return CommandUtils.withName("IntakeFromCS", Commands
            .parallel(r.elevator.RaiseToCSIntake(), r.arm.PivotToStowed())
            .andThen(r.launcher.cmdIntakeCoral()));
    }

    public final Command ScoreCoralL3() 
    {
        return CommandUtils.withName("ScoreCoralL3", 
            r.arm.PivotTo180()
            .andThen(r.launcher.cmdScoreCoral().withTimeout(0.3)));
    }

    public final Command ScoreCoraL4() 
    {
        return CommandUtils.withName("ScoreCoralL4", 
            Commands.parallel(r.elevator.RaiseToL4(), r.arm.PivotTo180())
            .andThen(r.launcher.cmdScoreCoral().withTimeout(0.3)));
    }

    public final Command AlignCoralL3() 
    {
        return CommandUtils.withName("AlignCoralL3", r.elevator.RaiseToStowed().alongWith(r.arm.PivotTo180()));
    }

    public final Command AlignCoralL4()
    {
        return CommandUtils.withName("AlignCoralL4", r.elevator.RaiseToL4().alongWith(r.arm.PivotTo180()));
    }

    public final Command RaiseToMax()
    {
        return CommandUtils.withName("GoToMax",
        Commands.parallel(r.arm.PivotTo180())
        .andThen(r.elevator.RaiseToL4()));
    }


    private final Robot r;

    public GameCommands(Robot r)
    {
        this.r = r;
    }    
}
