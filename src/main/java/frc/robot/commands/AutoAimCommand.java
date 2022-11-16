// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotCharacterization;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Uses PhotonVision to turn the robot toward an acquired target
 */
public class AutoAimCommand extends CommandBase {

  DriveSubsystem m_driveSubsystem;
  PhotonCamera m_camera; 

  PIDController m_turnPIDController = new PIDController(RobotCharacterization.kAngularP, 0, RobotCharacterization.kLinearD);

  /** Creates a new AutoAimCommand. */
  public AutoAimCommand(DriveSubsystem driveSubsystem, PhotonCamera camera) {
    m_driveSubsystem = driveSubsystem;
    m_camera = camera;

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
        // Calculate angular turn power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -m_turnPIDController.calculate(result.getBestTarget().getYaw(), 0);
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
