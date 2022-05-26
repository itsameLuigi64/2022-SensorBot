// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class SensorArraySubsystem extends SubsystemBase {

  // Sensor Constants
  private static double kUltraSonicScaleFactor = 1024.0;

  private DigitalInput m_beamBreak = null;
  private AnalogInput m_ultraSonic = null;

  /** Creates a new SensorArraySubsystem. */
  public SensorArraySubsystem() {
    // Sensors (uncomment to exclude from the robot)
    m_beamBreak = new DigitalInput(RoboRio.DioPort.kBeamBreak);
    m_ultraSonic = new AnalogInput(RoboRio.AnalogPort.kUltraSonic);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_beamBreak != null) {
      SmartDashboard.putBoolean("Beam Break", m_beamBreak.get());
    }
    if (m_ultraSonic != null) {
      double voltage = m_ultraSonic.getVoltage();

      SmartDashboard.putNumber("UltraSonic Voltage", voltage);
      SmartDashboard.putNumber("UltraSonic Distance", voltage * kUltraSonicScaleFactor);
    }
  }

  public void reset() {
    // Add any necessary reset code here
  }
}
