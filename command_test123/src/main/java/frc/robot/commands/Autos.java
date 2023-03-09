// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * 
 * This file is outdated and no longer supported. Refer to frc.robot.commands.Autonomous for current code.
 * 
 */

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.AutosSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAuto(AutosSubsystem subsystem) {
    // Prints Beginning Autos when switched to autonomous mode

    return Commands.sequence(subsystem.exampleMethodCommand()/* , new ExampleCommand()*/);
  }

  public Autos(Drivetrain drive) {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}