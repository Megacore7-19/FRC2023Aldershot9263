// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** The main autonomous command to pickup and deliver the soda to the box. */
public class Autonomous extends SequentialCommandGroup {
  /** Create a new autonomous command. */
  public Autonomous(Drivetrain drive) {
    addCommands(
      new DriveStraight(0.1, drive),
      new WaitCommand(2.0),
      new DriveStraight(-0.1, drive),
      new WaitCommand(1),
      new DriveStraight(0.1, drive),
      new WaitCommand(1),
      new DriveStraight(0, drive)
      
      // // moves and scores game piece
      //   new DriveStraight(-0.1, drive),
      // // goes outside community zone
      //   new DriveStraight(5.35, drive),
      // // Goes middle of charge station
      //   new DriveStraight(4.15, drive)
        );
  }
}