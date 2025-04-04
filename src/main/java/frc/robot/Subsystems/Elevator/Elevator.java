package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkConfigurations;
import frc.robot.Commands.CommandUtils;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {

    public static final double kMaxHeight = 23;
    public static final double kMinHeight = 0;

    private static final double kGearing = 20;
    private static final double kDiameter = 1.1279;
    private static final double kCircumference = kDiameter * Math.PI;

    // This is 1 / [2048 Cycles per Revolution (Pulse = One Revolution)]
    // https://www.revrobotics.com/rev-11-1271/
    private static final double distancePerPulse = 1f / 2048f;

    private final SparkMax elevatorMotor;
    private final Encoder elevatorEncoder;

    private final ProfiledPIDController PID = new ProfiledPIDController(0.20, 0, 0, new Constraints(45, 45));
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.01, 0.15, 1.5, 0);

    /**
     * Desired elevator height in inches!
     */
    private double targetPosition = 0;

    public static final double kL1Height = 5;
    public static final double kL2Height = 3;
    public static final double kL3Height = 17;
    public static final double kL4Height = 23;
    public static final double kStowHeight = 0;
    public static final double kLowerAlgaeHeight = 11;
    public static final double kNetHeight = 22;
    public static final double kCSIntakeHeight = 20;
    public static final double lolipop = 2.5;

    public Command RaiseToL1() { return CommandUtils.withName("RaiseToL1", makeRaiseToCommand(kL1Height)); }
    public Command RaiseToL2() { return CommandUtils.withName("RaiseToL2", makeRaiseToCommand(kL2Height)); }
    public Command RaiseToL3() { return CommandUtils.withName("RaiseToL3", makeRaiseToCommand(kL3Height)); };
    public Command RaiseToL4() { return CommandUtils.withName("RaiseToL4", makeRaiseToCommand(kL4Height)); }
    public Command RaiseToStowed() { return CommandUtils.withName("RaiseToStowed", makeRaiseToCommand(kStowHeight)); }
    public Command RaiseToLowerAlgae() { return CommandUtils.withName("RaiseToLowerAlgae", makeRaiseToCommand(kLowerAlgaeHeight)); }
    public Command RaiseToNet() { return CommandUtils.withName("RaiseToNet", makeRaiseToCommand(kNetHeight)); } 
    public Command RaiseToCSIntake() { return CommandUtils.withName("RaiseToCSIntake", makeRaiseToCommand(kCSIntakeHeight)); }

    public void Stow()
    {
        this.setPositions(kStowHeight);
    }

    public void lolipop()
    {
        this.setPositions(lolipop);
    }

    public Elevator(int driveMotorDeviceId, int encoderDevicePortA, int encoderDevicePortB) {

        this.elevatorMotor = new SparkMax(driveMotorDeviceId, MotorType.kBrushless);
        this.elevatorEncoder = new Encoder(0, 1, true);
        // this.elevatorEncoder.setDistancePerPulse(0.00048828125);
        
        this.PID.setTolerance(1);

        SparkConfigurations.ApplyConfigPersistNoReset(elevatorMotor, SparkConfigurations.CoastMode);

        SmartDashboard.putData("Elevator/PID", this.PID);
    }

    public Command makeRaiseToCommand(double heightInInches)
    {
        return Commands.run(() -> this.setPositions(heightInInches), this).until(() -> this.isAtHeight());
    }

    /**
     * @return The current height of the elevator, in inches.
     */
    public double getHeight()
    {
        return this.elevatorEncoder.getDistance() * kCircumference * -1 * distancePerPulse;
    }

    public void setPositions(double heightInInches) 
    {
        this.targetPosition = heightInInches;
        this.PID.setGoal(heightInInches);
    }

    public void stop()
    {
        this.elevatorMotor.stopMotor();
        this.PID.reset(this.getHeight());
        this.setPositions(this.getHeight());
    };

    public boolean isAtHeight() {
        return this.PID.atGoal();
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) return;

     //   SmartDashboard.putNumber("Elevator/Amps", this.elevatorMotor.getOutputCurrent());

        double PIDEffort = this.PID.calculate(this.getHeight(), this.targetPosition);

        SmartDashboard.putNumber("Elevator/PIDEffort", this.elevatorMotor.getOutputCurrent());

        if (this.getHeight() >= kMaxHeight && PIDEffort > 0) {

            this.elevatorMotor.stopMotor();
            return;
        }

        if (this.getHeight() <= kMinHeight && PIDEffort < 0) {

            this.elevatorMotor.stopMotor();
            return;
        }

        double voltage = feedforward.calculate(PIDEffort) * kGearing;

        this.elevatorMotor.setVoltage(voltage);

   //     SmartDashboard.putNumber("Elevator/Voltage", voltage);

    }
}