package frc.robot.Subsystems.LEDs;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.LEDPattern;

public class LEDProgram {
    private final LEDPattern primaryColor;
    private final Supplier<LEDPattern> colorSupplier;
    public final Optional<Double[]> togglePattern; // Changed type to Optional<Double[]>
    public final boolean doLoopTogglePattern;

    public LEDProgram(LEDPattern primaryColor)
    {
        this.primaryColor = primaryColor;
        this.colorSupplier = null;
        this.togglePattern = Optional.empty();
        this.doLoopTogglePattern = false;
    }

    public LEDProgram(LEDPattern primaryColor, String togglePattern, boolean doLoopTogglePattern)
    {
        this.primaryColor = primaryColor;
        this.colorSupplier = null;
        this.togglePattern = Optional.of(parseStringTogglePattern(togglePattern));
        this.doLoopTogglePattern = doLoopTogglePattern;
    }

    /**
     * Constructs an LED state with a static color pattern.
     *
     * @param colorSupplier The LED pattern supplier.
     */
    public LEDProgram(Supplier<LEDPattern> colorSupplier) {
        this.primaryColor = null;
        this.colorSupplier = colorSupplier;
        this.togglePattern = Optional.empty();
        this.doLoopTogglePattern = false;
    }

    /**
     * Constructs an LED state with a toggle pattern.
     *
     * @param colorSupplier The LED pattern supplier.
     * @param togglePattern A comma-separated list of time intervals (in seconds) controlling LED toggling.
     *                      Example: "0, 0.2" will toggle the LEDs off at 0s and on at 0.2s.
     * @param doLoopTogglePattern Whether the toggle pattern should loop indefinitely.
     * @throws IllegalArgumentException if togglePattern has fewer than two time intervals.
     */
    public LEDProgram(Supplier<LEDPattern> colorSupplier, String togglePattern, boolean doLoopTogglePattern) {
        this.primaryColor = null;
        this.colorSupplier = colorSupplier;
        this.togglePattern = Optional.of(parseStringTogglePattern(togglePattern));
        this.doLoopTogglePattern = doLoopTogglePattern;
    }

    public LEDPattern getColors()
    {
        return this.primaryColor == null ? this.colorSupplier.get() : this.primaryColor;
    }

    private static Double[] parseStringTogglePattern(String stringTogglePattern)
    {
        String[] splitPattern = stringTogglePattern.split(",");
        if (splitPattern.length < 2) {
            throw new IllegalArgumentException("Argument togglePattern must have at least two time intervals!");
        }

        // Convert String[] to Double[]
        return Arrays.stream(splitPattern)
            .map(v -> Double.parseDouble(v.trim()))
            .toArray(Double[]::new);
    }
}
