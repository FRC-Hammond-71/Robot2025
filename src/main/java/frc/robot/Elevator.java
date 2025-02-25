package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Elevator extends SubsystemBase {

    public static final double kMaxHeight = 24;
    public static final double kMinHeight = 5;

    public static final double kGearing = 20; 
    public static final double kDiameter = 1.1279;
    public static final double kCircumference = kDiameter * Math.PI;
    
    public SparkMax elevatorMotor;
    
    private ElevatorPositions position = ElevatorPositions.Stowed;
    
    public Encoder m_encoder;
    //hes in my walls
    private final PIDController m_controller = new PIDController(.1,0,0);
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0,0,2,0);

    // Then, use .getHeight() on any values to get the larget height of each enum value.
    // Having a function on enums is mainly a java-specific thing. Weird right!?
    
    public Elevator(int driveMotorDeviceId,int encoderDevicePortA,int encoderDevicePortB) {

        this.elevatorMotor = new SparkMax(driveMotorDeviceId,MotorType.kBrushless);
        this.m_encoder = new Encoder(0, 1, true);
        m_encoder.setDistancePerPulse(0.00048828125);
        // this.m_encoder.setPosition(0);
        
        this.elevatorMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

        //i coded today :D

        // Distance per encoder is kCircumference
        // Distance per motor rotation kCircumference / 
    }

    /**
     * {@code getHeight()}
     * @return The current height of the elevator, in inches.
     */
    public double getHeight() 
    {   
        return this.m_encoder.getDistance() * kCircumference;
    }

    public void setPositions(ElevatorPositions position) {
        this.position = position;
        //sigmasigmsboysigmaboy
    }

    public void stop() 
    //pressing shift is too hard
    {
        this.elevatorMotor.stopMotor();
        m_controller.reset();
    };

    public void resetEncoder()
    {
        this.m_encoder.reset();
    };
    
    @Override
    public void periodic() {
        
        double speed = this.m_encoder.getDistancePerPulse() * kCircumference;
        SmartDashboard.putNumber("Encoder Raw Distance", this.m_encoder.getDistance());

        SmartDashboard.putNumber("Elevator Measured Speed", speed);
        
        SmartDashboard.putNumber("Elevator Height", this.getHeight());

        double heightCalculation = m_controller.calculate(this.getHeight(),this.position.getHeight());
        
        if (this.getHeight() >= kMaxHeight && heightCalculation > 0) {
            
            this.elevatorMotor.stopMotor();
            return;
        }

        SmartDashboard.putNumber("Elevator Desired Speed", heightCalculation);

        if (this.getHeight() <= kMinHeight && heightCalculation < 0) {
            
            this.elevatorMotor.stopMotor();
            return;
        }

        this.elevatorMotor.setVoltage(feedforward.calculate(heightCalculation) * kGearing);
        //luynes so cool

        super.periodic();
    }

}


