// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Autonomous;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Drivetrain code
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.TankDrive;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XboxController m_joystick = new XboxController(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  private final CommandBase m_autonomousCommand =
      new Autonomous(m_drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(
        new TankDrive(() -> m_joystick.getLeftY(), () -> -m_joystick.getLeftX(), m_drivetrain));
 
    // Show what command your subsystem is running on the SmartDashboard 
    SmartDashboard.putData(m_drivetrain);
 
    // Configure the button bindings    
    configureBindings();
  }

  public void log() { }
  private void configureBindings() { /*Create some buttons */ }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }
}
