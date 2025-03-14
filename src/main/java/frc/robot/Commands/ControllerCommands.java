package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControllerCommands {
    
    public static final Command Rumble(XboxController controller)
    {
        return Rumble(controller, 0.3);
    }
    public static final Command Rumble(XboxController controller, double durationInSeconds)
    {
        return CommandUtils.withName("Rumble Controller", Commands.runEnd(
            () -> controller.setRumble(RumbleType.kBothRumble, 1), 
            () -> controller.setRumble(RumbleType.kBothRumble, 0))
            .withTimeout(durationInSeconds));
    }

}
