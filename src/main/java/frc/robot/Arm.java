package frc.robot;

import org.dyn4j.geometry.Rotation;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase {

    private Rotation2d targetRotation;
    private ProfiledPIDController PID;
    private ArmFeedforward feedforward;
    private SparkMax sparkMax;
    private AbsoluteEncoder absoluteEncoder;

    
    public static final double kGearing = (17 / 3) * (15 / 4) * (20 / 7);
    public static final Rotation2d maximumRotation = Rotation2d.fromDegrees(180);
    private final PIDController m_controller = new PIDController(.1,0,0);

    public int ArmdriveMotorDeviceId;
    public int encoderDevicePortA;

    public Arm(int ArmdriveMotorDeviceId, int encoderDevicePortA){

        this.sparkMax = new SparkMax(ArmdriveMotorDeviceId, MotorType.kBrushless);
        this.targetRotation = new Rotation2d(); // In degrees goofy :)
        this.feedforward = new ArmFeedforward(0, 0, 3, 0);
        this.PID = new ProfiledPIDController(.1, 0, 1, new Constraints(30, 5));
    }
    
        public double calculate() {
            double calculated = feedforward.calculate(0, 0, 0);
            return calculated;
        }
        public Command ScoreCommand = Commands.run(() -> this.m_arm.Score(), this);
        public Command IntakeCommand = Commands.run(() -> this.m_arm.Intake(), this);
                private Arm m_arm = new Arm(ArmdriveMotorDeviceId, encoderDevicePortA);
                
                public void Intake() {
                this.m_arm.sparkMax.set(-1);
            }
            public void Score() {
                this.m_arm.sparkMax.set(1);
            }
        
        
            public void Stop() {
                this.m_arm.sparkMax.stopMotor();
    }

    // feedforward.calculate(1, 2, 3);
    @Override
    public void periodic() {
       super.periodic();
    }

}