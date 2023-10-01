// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.*;
import java.net.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Autonomous;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.DownElevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorMain;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.TeleopCamera;
// Drivetrain code
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.TankDrive;
import frc.robot.commands.UpElevator;

// Experimental Simulation Code

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
  private final Joystick m_joystick = new Joystick(0);
  private final XboxController m_controllerPrimary = new XboxController(0);
  private final ElevatorMain m_elevator = new ElevatorMain();
  private final TeleopCamera m_camera = new TeleopCamera();
  private ServerSocket serverSocket;
  private Socket unitySocket;
  private int localPortNum = 5810;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandBase m_autonomousCommand =
      new Autonomous(m_drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(
        new TankDrive(() -> m_controllerPrimary.getRawAxis(0), () -> -m_controllerPrimary.getRawAxis(1), m_drivetrain));
 
    // Show what command your subsystem is running on the SmartDashboard 
    SmartDashboard.putData(m_drivetrain);
    SendableRegistry.setName(m_drivetrain, "Drivetrain", "DrivetrainChassis");
    SmartDashboard.putData(m_autonomousCommand);
    SmartDashboard.putData(m_elevator);
    SmartDashboard.putData(m_claw);
    SmartDashboard.putData(m_camera);
    // Configure the button bindings    
    configureBindings();
    if (RobotBase.isSimulation()) {
      System.out.println("Establishing Simulator Bridge");
      establishSimBridge();
    }
  }

  public void log() {}

  private void configureBindings() { 
    /*Create some buttons */ 
  
    final JoystickButton dPadUp = new JoystickButton(m_joystick, 1);
    final JoystickButton dPadDown = new JoystickButton(m_joystick, 2);
    final JoystickButton dPadRight = new JoystickButton(m_joystick, 3);
    final JoystickButton dPadLeft = new JoystickButton(m_joystick, 4);
 
    // Connect the buttons to commands
    dPadUp.whileTrue(new OpenClaw(m_claw));
    dPadDown.whileTrue(new CloseClaw(m_claw));
    dPadRight.whileTrue(new UpElevator(m_elevator, 1));
    dPadLeft.whileTrue(new DownElevator(m_elevator, 1));
  }

  private void establishSimBridge() {
    try {
      serverSocket = new ServerSocket(localPortNum);
      System.out.println("Waiting for Unity connection on port " + localPortNum);
      unitySocket = serverSocket.accept();
      System.out.println("Unity connected!");

      // Start a thread for sending data to Unity
      Thread sendToUnityThread = new Thread(() -> {
          try {
              PrintWriter out = new PrintWriter(unitySocket.getOutputStream(), true);
              BufferedReader in = new BufferedReader(new InputStreamReader(unitySocket.getInputStream()));
              
              // Send data to Unity
              while (true) {
                  // String dataToSend = "[{" + 
                  //   "\"clawLeft\": " + "\"" + SmartDashboard.getNumber("Claw - Left", 0) + "\"," + 
                  //   "\"elevatorRight\": " + "\"" + SmartDashboard.getNumber("Elevator - Right", 0) + "\"," + 
                  //   "\"drivetrainLeft\": " + "\"" + m_drivetrain.m_leftMotor.get() + "\"," + 
                  //   "\"drivetrainRight\": " + "\"" + m_drivetrain.m_rightMotor.get() + "\""
                  //   + "}]";
                  String dataToSend = "" + 
                  "clawLeft:" + SmartDashboard.getNumber("Claw - Left", 0) + 
                  "/elevatorRight:" +  SmartDashboard.getNumber("Elevator - Right", 0) +
                  "/drivetrainLeft:" + m_drivetrain.m_leftMotor.get() +
                  "/drivetrainRight:" + m_drivetrain.m_rightMotor.get()
                  ;

                  out.println(dataToSend);
                  Thread.sleep(20); // Adjust the interval as needed

                  // String receivedData = in.readLine();
                  // if (receivedData != null) {
                  //     // Process received data as needed
                  //     System.out.println("Received data from Unity: " + receivedData);
                  // }
              }
          } catch (IOException | InterruptedException e) {
              e.printStackTrace();
          }
      });
      sendToUnityThread.start();
      sendToUnityThread.setName("SimulatorServerThread");
      sendToUnityThread.setPriority(Thread.MIN_PRIORITY);
  } catch (IOException e) {
      e.printStackTrace();
  }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }
}
