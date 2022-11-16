// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Field;
import frc.robot.Constants.RobotCharacterization;
import frc.robot.Constants.RobotConfig;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Uses PhotonVision to drive the robot to a specified distance from an acquired target 
 */
public class AutoRangeCommand extends CommandBase {
  
  DriveSubsystem m_driveSubsystem;
  PhotonCamera m_camera; 
  final double m_goalRangeInMeters;
    
  PIDController m_forwardPIDController = new PIDController(RobotCharacterization.kLinearP, 0, RobotCharacterization.kLinearD);

  /** Creates a new AutoRangeCommand. */
  public AutoRangeCommand(DriveSubsystem driveSubsystem, PhotonCamera camera, double goalRangeInMeters) {
    m_driveSubsystem = driveSubsystem;
    m_camera = camera;
    m_goalRangeInMeters = goalRangeInMeters;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sit still if no target was found    
    double forwardSpeed = 0;
    double rotationSpeed = 0;

    // Query the latest result from PhotonVision
    var result = m_camera.getLatestResult();

    if (result.hasTargets()) {
        // First calculate current range
        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                  RobotConfig.Camera.kHeightInMeters,
                  Field.Target1.kHeightInMeters,
                  RobotConfig.Camera.kPitchInRadians,
                  Units.degreesToRadians(result.getBestTarget().getPitch()));

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        forwardSpeed = -m_forwardPIDController.calculate(range, m_goalRangeInMeters);        
    }    
    m_driveSubsystem.arcadeDrive(forwardSpeed, rotationSpeed);

  }
        

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
