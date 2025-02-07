// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.io.Console;
//import java.lang.ModuleLayer.Controller;

//import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final LinearFilter xFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
  private final LinearFilter yFilter = LinearFilter.singlePoleIIR(0.3, 0.02);

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    m_swerve.dashboardPrint();  
    
    if (m_controller.getYButton()) {
      this.m_swerve.GyroReset();
      System.out.println("GYRO RESET");
    }
  }

  @Override
  public void autonomousPeriodic() {

    m_swerve.updateOdometry();
    m_swerve.updateFieldPosition();

  }



  @Override

  public void teleopPeriodic() {

    m_swerve.updateOdometry();
    m_swerve.updateFieldPosition();

    driveWithJoystick(true);

    driverControls();

  }

  private void driveWithJoystick(boolean fieldRelative) {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
    xFilter.calculate(Math.pow(MathUtil.applyDeadband(-m_controller.getLeftY(), 0.15), 3)) * Drivetrain.kMaxSpeed;
    
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        yFilter.calculate(Math.pow(MathUtil.applyDeadband(-m_controller.getLeftX(), 0.15), 3)) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // final var rot =
        // -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.1)) * Drivetrain.kMaxAngularSpeed;
    
    // final var rot = Math.atan(-m_controller.getRightY() / m_controller.getRightX());
    double rot = MathUtil.applyDeadband(-m_controller.getRightX(), 0.15);

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,ySpeed,rot);

    // double rot = MathUtil.applyDeadband(-m_controller.getRightX(), 0.15) * Math.PI / 2;

    //rot = Math.atan2(MathUtil.applyDeadband(-m_controller.getRightY(), 0.15), MathUtil.applyDeadband(m_controller.getRightX(), 0.15));
    
    // if (rot < 0)
    // {
    //   rot += Math.PI * 2;
    // }

    // if (m_controller.getAButton()) {
    //   rot = Math.PI;
    // }
    // else  if (m_controller.getXButton()) {
    //   rot = Math.PI / 2;
    // }
    // else  if (m_controller.getBButton()) {
    //   rot =  (3 * Math.PI) / 2;
    // }
    // else {
    //   rot = 0;
    // }

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", Math.toDegrees(rot));
    m_swerve.Drive(speeds, fieldRelative);
    //m_swerve.Drive(1, 0, 0, fieldRelative, getPeriod());
  }

  private void driverControls() {
    //(reserved for climb))
    if (m_controller.getAButtonPressed()) {

    }
    //(reserved for climb)
    if (m_controller.getYButtonPressed()) {
      m_swerve.GyroReset();
    }
    // Open intake X
    if (m_controller.getXButtonPressed()) {
      
    }
    //  Close intake B
    if (m_controller.getBButtonPressed()) {
      
    }
  }
}
