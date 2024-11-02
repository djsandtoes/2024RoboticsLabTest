// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.MotorSpeedConstants;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;
  private double m_target_distance;
  private double m_current_distance;
  private double m_distance_tolerance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;
    //m_target_distance = SmartDashboard.getNumber("Target Distance", 10000.0);
    //m_distance_tolerance = SmartDashboard.getNumber("Target Tolerance", 1000.0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_target_distance = SmartDashboard.getNumber("Target Distance", 10000.0);
    m_distance_tolerance = SmartDashboard.getNumber("Target Tolerance", 1000.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_current_distance=m_subsystem.GetDistance();
    if (m_current_distance<m_target_distance)
      m_subsystem.SetSpeed(MotorSpeedConstants.CRAWL_FORWARD);
    else if (m_current_distance>m_target_distance)
      m_subsystem.SetSpeed(MotorSpeedConstants.CRAWL_BACKWARD);
    else 
      m_subsystem.SetSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.SetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_current_distance-m_target_distance)<m_distance_tolerance)
      return true;
    else
      return false;
  }
}
