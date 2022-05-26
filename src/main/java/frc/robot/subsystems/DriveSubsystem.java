// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

import static frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {

  // Drive constants
  private static final double kClosedLoopRampRate = 0.3;
  /* The following is an estimate. We need to confirm with the actual robot and remove this comment. */
  private static final double kDistancePerRotation = 2 * Math.PI * 6.0;

  private static final double kDriveScaleFactor = 0.80;
  private static final double kTurnScaleFactor = 0.70;

  private static final double kCollisionThresholdDeltaG = 0.5;

  private CANSparkMax m_leftLeader;
  private CANSparkMax m_leftFollower;  
  private CANSparkMax m_rightLeader;
  private CANSparkMax m_rightFollower;

  private DifferentialDrive m_diffDrive;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  // The gyro sensor
  private AHRS m_navx;

  // Class variable for collision detection
  private double m_lastLinearAccelY;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftLeader = new CANSparkMax(RoboRio.CanID.kLeftLeader, MotorType.kBrushless);
    m_leftFollower = new CANSparkMax(RoboRio.CanID.kLeftFollower, MotorType.kBrushless);
    m_rightLeader = new CANSparkMax(RoboRio.CanID.kRightLeader, MotorType.kBrushless);
    m_rightFollower = new CANSparkMax(RoboRio.CanID.kRightFollower, MotorType.kBrushless);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    m_rightLeader.setInverted(true);

    setClosedLoopRampRate(kClosedLoopRampRate);

    m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    // m_leftLeader.setSmartCurrentLimit(30, 90, 10);
    // m_leftFollower.setSmartCurrentLimit(30, 90, 10);
    // m_rightLeader.setSmartCurrentLimit(30, 90, 10);
    // m_rightFollower.setSmartCurrentLimit(30, 90, 10);

    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();    

    m_leftEncoder.setPositionConversionFactor(kDistancePerRotation);
    m_rightEncoder.setPositionConversionFactor(kDistancePerRotation);

    try {
      m_navx = new AHRS(SPI.Port.kMXP);
      SmartDashboard.putData(m_navx);
    }
    catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MSP: " + ex.getMessage(), true);
    }
  }

  @Override  
  public void periodic() {
    // This method will be called once per scheduler run
    //detectCollision();

    SmartDashboard.putNumber("Left Encoder Distance", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Distance", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Drive Motor Speed", m_leftLeader.get());
    SmartDashboard.putNumber("Right Drive Motor Speed", m_rightLeader.get());

  }

  public void setClosedLoopRampRate(double rate) {
    this.m_leftLeader.setClosedLoopRampRate(rate);
    this.m_rightLeader.setClosedLoopRampRate(rate);
  }
  
  public void setIdleMode(IdleMode mode) {
		m_leftLeader.setIdleMode(mode);
		m_leftFollower.setIdleMode(mode);
		m_rightLeader.setIdleMode(mode);
		m_rightFollower.setIdleMode(mode);
	}

  public void resetEncoders() {
    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);
  }

  public double getLeftDistance() {
    return m_leftEncoder.getPosition();
  }

  public double getRightDistance() {
    return m_rightEncoder.getPosition();
  }

  public void stop() {
    m_diffDrive.tankDrive(0, 0);
  }

  public void arcadeDrive(double forwardSpeed, double rotation) {
    m_diffDrive.arcadeDrive(forwardSpeed*kDriveScaleFactor, rotation*kTurnScaleFactor);
  }
  
  public void arcadeDriveWithThrottle(double forwardSpeed, double rotation, double throttle) {
    final double kTurnRatio = 0.85;

    // re-scale throttle value to be from 0 to 1
    throttle = (throttle + 1)/ 2;

    forwardSpeed *= throttle;
    rotation *= throttle * kTurnRatio;

    m_diffDrive.arcadeDrive(forwardSpeed, rotation);
  }

  public void detectCollision() {
    boolean collisionDetectedY = false;

    if (m_navx != null) {
      double currentLinearAccelY = m_navx.getWorldLinearAccelY();
      double currentJerkY = currentLinearAccelY - m_lastLinearAccelY;
      m_lastLinearAccelY = currentLinearAccelY;

      if (Math.abs(currentJerkY) > kCollisionThresholdDeltaG) {
        collisionDetectedY = true;
      }
    }

    SmartDashboard.putBoolean("Collision Detected Y", collisionDetectedY);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }
}
