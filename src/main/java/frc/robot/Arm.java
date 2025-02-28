package frc.robot;

import org.dyn4j.geometry.Rotation;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

        private Rotation2d targetRotation;
        private ProfiledPIDController PID;
        private ArmFeedforward feedforward;
        private SparkMax sparkMax;
        private AbsoluteEncoder absoluteEncoder;

        
        public static final double kGearing = (17/3)*(15/4)*(20/7);
        public static final Rotation2d maximumRotation = Rotation2d.fromDegrees(180); 

        public Arm(int ArmdriveMotorDeviceId,int encoderDevicePortA) {
            
            this.sparkMax = new SparkMax(ArmdriveMotorDeviceId, MotorType.kBrushless);
            this.targetRotation = new Rotation2d(); //In degrees goofy :)
            this.feedforward = new ArmFeedforward(0,0,3,0);
            this.PID = new ProfiledPIDController(.1, 0, 1, new Constraints(30, 5)); 
        }

        public double calculate() {
           double calculated = feedforward.calculate(0, 0,0);
           return calculated;
        } 

        public void Intake() {
            this.sparkMax.set(-1);
        }


        public void Score() {
            this.sparkMax.set(1);
        }

        public void Stop() {
            this.sparkMax.stopMotor();
        }
       // feedforward.calculate(1, 2, 3);
        @Override
        public void periodic() {
            
        super.periodic();
        }
}
