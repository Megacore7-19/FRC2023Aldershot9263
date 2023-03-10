// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorMain;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Opens the claw for one second. Real robots should use sensors, stalling motors is BAD! */
public class UpElevator extends WaitCommand {
  private final ElevatorMain m_elevator;

  /**
   * Creates a new OpenClaw command.
   *
   * @param elevatorMotor The claw to use
   */
  public UpElevator(ElevatorMain elevatorMotor) {
    super(1.5);
    m_elevator = elevatorMotor;
    addRequirements(m_elevator);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_elevator.open();
    super.initialize();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }
}