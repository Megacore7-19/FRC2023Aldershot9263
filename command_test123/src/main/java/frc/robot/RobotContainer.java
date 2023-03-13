// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Autonomous;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.CloseClaw;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Claw;
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
  public final Drivetrain m_drivetrain = new Drivetrain();
  private final Claw m_claw = new Claw();
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
  private void configureBindings() { 
    /*Create some buttons */ 
  
    final JoystickButton dPadUp = new JoystickButton(m_joystick, 1);
    final JoystickButton dPadDown = new JoystickButton(m_joystick, 2);
    // final JoystickButton dPadRight = new JoystickButton(m_joystick, 3);
    // final JoystickButton dPadDown = new JoystickButton(m_joystick, 2);
    // final JoystickButton dPadLeft = new JoystickButton(m_joystick, 1);
    // final JoystickButton l2 = new JoystickButton(m_joystick, 5);
    // final JoystickButton r2 = new JoystickButton(m_joystick, 6);
    // final JoystickButton l1 = new JoystickButton(m_joystick, 7);
    // final JoystickButton r1 = new JoystickButton(m_joystick, 8);
 
    // Connect the buttons to commands
    dPadUp.whileTrue(new OpenClaw(m_claw));
    dPadDown.whileTrue(new CloseClaw(m_claw));
  }
    // dPadUp.whileTrue(new ExampleCommand());}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }
}
