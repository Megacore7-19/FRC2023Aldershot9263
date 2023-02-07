// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// Drivetrain code
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.TankDrive;
// Controls code
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.examples.gearsbot.commands.Autonomous;
// import edu.wpi.first.wpilibj.examples.gearsbot.commands.CloseClaw;
// import edu.wpi.first.wpilibj.examples.gearsbot.commands.OpenClaw;
// import edu.wpi.first.wpilibj.examples.gearsbot.commands.Pickup;
// import edu.wpi.first.wpilibj.examples.gearsbot.commands.Place;
// import edu.wpi.first.wpilibj.examples.gearsbot.commands.PrepareToPickup;
// import edu.wpi.first.wpilibj.examples.gearsbot.commands.SetElevatorSetpoint;
// import edu.wpi.first.wpilibj.examples.gearsbot.commands.SetWristSetpoint;
// import edu.wpi.first.wpilibj.examples.gearsbot.commands.TankDrive;
// import edu.wpi.first.wpilibj.examples.gearsbot.subsystems.Claw;
// import edu.wpi.first.wpilibj.examples.gearsbot.subsystems.Drivetrain;
// import edu.wpi.first.wpilibj.examples.gearsbot.subsystems.Elevator;
// import edu.wpi.first.wpilibj.examples.gearsbot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XboxController m_joystick = new XboxController(0);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(
        new TankDrive(() -> -m_joystick.getLeftY(), () -> -m_joystick.getRightY(), m_drivetrain));
 
    // Show what command your subsystem is running on the SmartDashboard
    SmartDashboard.putData(m_drivetrain);
 
    // Configure the button bindings    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Create some buttons
    final JoystickButton dpadUp = new JoystickButton(m_joystick, 5);
    final JoystickButton dpadRight = new JoystickButton(m_joystick, 6);
    final JoystickButton dpadDown = new JoystickButton(m_joystick, 7);
    final JoystickButton dpadLeft = new JoystickButton(m_joystick, 8);
    final JoystickButton l2 = new JoystickButton(m_joystick, 9);
    final JoystickButton r2 = new JoystickButton(m_joystick, 10);
    final JoystickButton l1 = new JoystickButton(m_joystick, 11);
    final JoystickButton r1 = new JoystickButton(m_joystick, 12);
 
    // Connect the buttons to commands
    dpadUp.onTrue(new ExampleCommand(m_exampleSubsystem));

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
