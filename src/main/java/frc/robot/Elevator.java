package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Elevator extends SubsystemBase {

    public static final double kMaxHeight = 22;
    public static final double kMinHeight = 5;

    private static final double kGearing = 20;
    private static final double kDiameter = 1.1279;
    private static final double kCircumference = kDiameter * Math.PI;

    // This is 1 / [2048 Cycles per Revolution (Pulse = One Revolution)]
    // https://www.revrobotics.com/rev-11-1271/
    private static final double distancePerPulse = 1f / 2048f;

    private final SparkMax elevatorMotor;
    private final Encoder elevatorEncoder;

    private final PIDController PID = new PIDController(0.2, 0, 0.0005);
    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0.5, 0);

    private ElevatorPosition targetPosition = ElevatorPosition.Stowed;

    public Command RaiseToL1 = Commands.run(() -> this.setPositions(ElevatorPosition.L1), this)
        .until(() -> this.isAtHeight());
    public Command RaiseToL2 = Commands.run(() -> this.setPositions(ElevatorPosition.L2), this)
        .until(() -> this.isAtHeight());
    public Command RaiseToL3 = Commands.run(() -> this.setPositions(ElevatorPosition.L3), this)
        .until(() -> this.isAtHeight());
    public Command RaiseToL4 = Commands.run(() -> this.setPositions(ElevatorPosition.L4), this)
        .until(() -> this.isAtHeight());
    public Command RaiseToStow = Commands.run(() -> this.setPositions(ElevatorPosition.Stowed), this)
        .until(() -> this.isAtHeight());

    public Elevator(int driveMotorDeviceId, int encoderDevicePortA, int encoderDevicePortB) {

        this.elevatorMotor = new SparkMax(driveMotorDeviceId, MotorType.kBrushless);
        this.elevatorEncoder = new Encoder(0, 1, true);
        // this.elevatorEncoder.setDistancePerPulse(0.00048828125);
        // this.m_encoder.setPosition(0);

        SparkConfigurations.ApplyConfigPersistNoReset(elevatorMotor, SparkConfigurations.CoastMode);
    }

    /**
     * @return The current height of the elevator, in inches.
     */
    public double getHeight()
    {
        return this.elevatorEncoder.getDistance() * kCircumference * -1 * distancePerPulse;
    }

    public void setPositions(ElevatorPosition position) 
    {
        this.targetPosition = position;
    }

    public void stop()
    {
        this.elevatorMotor.stopMotor();
        this.PID.reset();
    };

    public boolean isAtHeight() {
        return this.PID.atSetpoint();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Elevator Height", this.getHeight());

        double PIDEffort = PID.calculate(this.getHeight(), this.targetPosition.getHeight());

        if (this.getHeight() >= kMaxHeight && PIDEffort > 0) {

            this.elevatorMotor.stopMotor();
            return;
        }

        if (this.getHeight() <= kMinHeight && PIDEffort < 0) {

            this.elevatorMotor.stopMotor();
            return;
        }

        SmartDashboard.putNumber("Elevator Measured Speed", -this.elevatorEncoder.getRate() * distancePerPulse);
        SmartDashboard.putNumber("Elevator Desired Speed", PIDEffort);

        this.elevatorMotor.setVoltage(feedforward.calculate(PIDEffort) * kGearing);
        // //luynes so cool
    }

}

