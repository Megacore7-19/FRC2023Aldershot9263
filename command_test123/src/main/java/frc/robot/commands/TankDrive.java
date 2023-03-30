// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

/** Have the robot drive tank style. */
public class TankDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  // variable for controlling the curves:
  // needs to be tuned to the resistance that the motors experience on carpet
  private double torque_resistance_threshold = 0.05;
  /**
   * Creates a new TankDrive command.
   *
   * @param left The control input for the left side of the drive
   * @param right The control input for the right sight of the drive
   * @param drivetrain The drivetrain subsystem to drive
   */
  public TankDrive(DoubleSupplier left, DoubleSupplier right, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_left = left;
    m_right = right;
    addRequirements(m_drivetrain);
  }


  // function to apply a curve to the joystick input with a minimum value
  private double applyCurve(double joystickPosition) {
    if (joystickPosition > 0) {
      return (1 - torque_resistance_threshold) * Math.pow(joystickPosition, 3) + torque_resistance_threshold;
    }
    else if (joystickPosition < 0 ) {
      return (1 - torque_resistance_threshold) * Math.pow(joystickPosition, 3) - torque_resistance_threshold;
    }

    //return 0 if joystick is 0
    return 0;
  }

  // add a deadzone to the controls, especially since it would over come the resistance at the minimum value
  private double deadzone(double value, double deadzone) {
    if (Math.abs(value) < deadzone) {
      return 0;
    }
    return value;
  }


  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double leftWheelsPower = (deadzone(m_left.getAsDouble(), 0.01));
    double rightWheelsPower = (deadzone(m_right.getAsDouble(), 0.01));
    
    m_drivetrain.drive(applyCurve(leftWheelsPower), applyCurve(rightWheelsPower));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }
}