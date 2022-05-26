// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {

  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forwardSpeed;
  private final DoubleSupplier m_rotation;

  /** Creates a new ArcadeDriveCommand. */
  public ArcadeDriveCommand(DriveSubsystem subsystem, DoubleSupplier forwardSpeed, DoubleSupplier rotation) {
    m_drive = subsystem;
    m_forwardSpeed = forwardSpeed;
    m_rotation = rotation;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_forwardSpeed.getAsDouble(), m_rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
