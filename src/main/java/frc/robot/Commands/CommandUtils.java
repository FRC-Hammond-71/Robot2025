package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandUtils {
    public static <T extends Command> T withName(String name, T command)
    {
        command.setName(name);
        return command;
    }
}
