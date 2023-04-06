// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Have the robot drive tank style. */
public class TankDriveStraight extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final double m_runInSeconds;
  private final double m_speed;
  private final Timer timer;

  /**
   * Creates a new TankDrive command.
   *
   * @param speed The control input for speed of the drive
   * @param drivetrain The drivetrain subsystem to drive
   */
  
  public TankDriveStraight(double speed, Drivetrain drivetrain, double runInSeconds) {
    m_drivetrain = drivetrain;
    m_speed = speed;
    m_runInSeconds = runInSeconds;
    addRequirements(m_drivetrain);
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double leftWheelsPower = (m_speed);
    double rightWheelsPower = (m_speed * 0.965);
    m_drivetrain.drive(leftWheelsPower, rightWheelsPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return timer.get() > m_runInSeconds;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }
}