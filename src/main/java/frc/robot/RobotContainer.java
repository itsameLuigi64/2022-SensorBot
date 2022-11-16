// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem m_driveSubsystem = null;
  private SensorArraySubsystem m_sensorArray = null;

  // Operator interface
  private Joystick m_gamePad = null;
  
  // Cameras
  private UsbCamera m_camera1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Subsystems (comment out to exclude a subsystem from the robot)
    m_driveSubsystem = new DriveSubsystem();
    //m_sensorArray = new SensorArraySubsystem();

    // Controllers (comment out to exclude a controller from the laptop)
    m_gamePad = new Joystick(Laptop.UsbPort.kGamePad);

    // Cameras (comment out to exclude a camera from the robot);
    //m_camera1 = CameraServer.startAutomaticCapture(0);
    if (m_camera1 != null) {
      m_camera1.setResolution(320, 240);
    }

    // Configure the button bindings
    configureButtonBindings();
    
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (m_driveSubsystem != null && m_gamePad != null) {

      new JoystickButton(m_gamePad, GamePad.Button.kRB)
        .whenPressed(() -> m_driveSubsystem.resetEncoders(), m_driveSubsystem)
      ;

      // Testing turning the robot
      new JoystickButton(m_gamePad, GamePad.Button.kX)
        .whileHeld(() -> m_driveSubsystem.arcadeDrive(0, 0.2), m_driveSubsystem)
        .whenReleased(() -> m_driveSubsystem.stop(), m_driveSubsystem)
      ;
      
      // Testing turning the robot
      new JoystickButton(m_gamePad, GamePad.Button.kY)
        .whileHeld(() -> m_driveSubsystem.arcadeDrive(0, -0.2), m_driveSubsystem)
        .whenReleased(() -> m_driveSubsystem.stop(), m_driveSubsystem)
      ;
    }

    if (m_sensorArray != null && m_gamePad != null) {
      new JoystickButton(m_gamePad, GamePad.Button.kLB)
        .whenPressed(() -> m_sensorArray.reset()
      );
    };
  }

    /**
   * Use this method to define the default commands for subsystems
   */
  private void configureDefaultCommands() {

    if (m_driveSubsystem != null && m_gamePad != null) {
      m_driveSubsystem.setDefaultCommand(new ScaledArcadeDriveCommand(m_driveSubsystem, 
        () -> m_gamePad.getRawAxis(GamePad.RightStick.kUpDown), 
        () -> m_gamePad.getRawAxis(GamePad.LeftStick.kLeftRight)
      ));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // No Autonomous needed at this time
    return null;
  }
}
