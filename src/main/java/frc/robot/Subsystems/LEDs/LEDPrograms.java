package frc.robot.Subsystems.LEDs;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;

public class LEDPrograms 
{
    public static final Distance LED_SPACING = edu.wpi.first.units.Units.Meters.of(1.0 / 60);

    public static final String WarningTogglePattern = "1,1";
    public static final String RapidBlinkTogglePattern = "0.08,0.08,0.08,0.08";
    public static final LEDPattern Rainbow = LEDPattern.rainbow(255, 200);

    public static LEDProgram Warning = new LEDProgram(
        LEDPattern.solid(Color.kRed),
        WarningTogglePattern,
        true);

    public static LEDProgram PoseResetComplete = new LEDProgram(
        LEDPattern.solid(Color.kBlueViolet),
        RapidBlinkTogglePattern,
        false);

    public static LEDProgram BeginAutoCommand = new LEDProgram(
        () -> Rainbow.scrollAtAbsoluteSpeed(edu.wpi.first.units.Units.InchesPerSecond.of(16), LED_SPACING),
        RapidBlinkTogglePattern,
        false
    );

    public static LEDProgram Idle = new LEDProgram(LEDPattern.solid(Color.kDimGray));
}
