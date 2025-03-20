package frc.robot.Subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private static LEDController defaultController;

    public static LEDController getDefault() {
        return defaultController;
    }

    private final AddressableLED LEDs;
    private final AddressableLEDBuffer LED_Buffer;
    private LEDProgram currentState = null;
    private int patternProgressIndex = 0;
    private double lastPatternChangeInSeconds = 0.0;
    private boolean isToggledOn = false;
    private boolean isThreadRunning = false;

    public final Mechanism2d ledPreview;
    private final MechanismLigament2d ledPreviewBar;

    /**
     * Singleton - Prevent multiple instances.
     */
    public LEDController(int portNumber, int ledCount, boolean useSeparateThread) {
        if (defaultController != null) {
            throw new IllegalStateException("LEDController instance already exists!");
        }

        this.LEDs = new AddressableLED(portNumber);
        this.LED_Buffer = new AddressableLEDBuffer(ledCount);

        this.LEDs.setLength(ledCount);
        this.LEDs.start();

        this.ledPreview = new Mechanism2d(4, 1);
        this.ledPreviewBar = new MechanismLigament2d("LED Bar", 3, 0);

        this.ledPreview.getRoot("Root", 0.5, 0.5).append(this.ledPreviewBar);

        defaultController = this;

        if (useSeparateThread)
        {
            this.isThreadRunning = true;
            new LEDThread().start();
        }
    }

    /**
     * Set the LED state and reset pattern index.
     */
    public void setProgram(LEDProgram state) {
        this.currentState = state;
        this.isToggledOn = true;
        this.patternProgressIndex = 0;
        this.lastPatternChangeInSeconds = Timer.getFPGATimestamp();
    }

    /**
     * Applies the given LED pattern.
     */
    private void setColors(LEDPattern colorPattern) {
        colorPattern.applyTo(this.LED_Buffer);
        this.LEDs.setData(this.LED_Buffer);
    }

    public void updateLEDs() {
        if (currentState == null)
            return;

        LEDPattern toUpdateLEDPattern;

        if (currentState.togglePattern.isPresent()) {
            double elapsedTime = Timer.getFPGATimestamp() - lastPatternChangeInSeconds;
            Double[] patternDurations = currentState.togglePattern.get();

            if (patternProgressIndex < patternDurations.length
                    && elapsedTime > patternDurations[patternProgressIndex]) {
                // Toggle LEDs on/off
                isToggledOn = !isToggledOn;
                lastPatternChangeInSeconds = Timer.getFPGATimestamp();
                patternProgressIndex++;

                if (!isToggledOn) 
                {
                    setColors(LEDPattern.kOff);
                }
            }

            // Loop pattern if required
            if (patternProgressIndex >= patternDurations.length && currentState.doLoopTogglePattern) {
                patternProgressIndex = 0;
                isToggledOn = true;
                lastPatternChangeInSeconds = Timer.getFPGATimestamp();
            }
        }

        if (isToggledOn) {
            setColors(currentState.getColors());

            ledPreviewBar.setColor(new Color8Bit("#e0b64a"));
        } else {
            ledPreviewBar.setColor(new Color8Bit("#000000"));
        }
    }

    class LEDThread extends Thread
    {
        @Override
        public void run()
        {
            while (isThreadRunning)
            {
                try
                {
                    Thread.sleep(100);
                }
                catch (InterruptedException ex)
                {
                    System.out.println("LED Thread has been stopped!");
                    isThreadRunning = false;
                }

                updateLEDs();
            }
        }
    }
}