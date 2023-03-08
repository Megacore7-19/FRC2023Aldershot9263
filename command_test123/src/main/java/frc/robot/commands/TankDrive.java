// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

/** Have the robot drive tank style. */
public class TankDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  /**
   * Creates a new TankDrive command.
   *
   * @param left The control input for the left side of the drive
   * @param right The control input for the right sight of the drive
   * @param drivetrain The drivetrain subsystem to drive
   */
  public TankDrive(DoubleSupplier left, DoubleSupplier right, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Left is the left wheels, or left joystick
    m_left = left;
    // Right is the right wheels, or right joystick
    m_right = right;
    addRequirements(m_drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // The formula for a curve that will ease into the values is as follows
    // y = (y - 2) * (-1 * x^2)
    // Or with y isolated
    // y = (2 * x^2) / (1 + x^2)
    double leftWheelsPower = (2 * Math.pow(m_left.getAsDouble(), 2)) / (1 + Math.pow(m_left.getAsDouble(), 2));
    double rightWheelsPower = (2 * Math.pow(m_right.getAsDouble(), 2)) / (1 + Math.pow(m_right.getAsDouble(), 2));
    m_drivetrain.drive(leftWheelsPower, rightWheelsPower);
    System.out.println("Left: " + m_left.getAsDouble() + "\t|\tRight: " + m_right.getAsDouble());
    System.out.println("Left: " + leftWheelsPower + "\t|\tRight: " + rightWheelsPower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }
}