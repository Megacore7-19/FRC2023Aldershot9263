// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Wrist;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Autos;

/** The main autonomous command to pickup and deliver the soda to the box. */
public class Autonomous extends SequentialCommandGroup {
  /** Create a new autonomous command. */
    //   public Autonomous(Drivetrain drive, Claw claw, Wrist wrist, Elevator elevator) {
  public Autonomous(Drivetrain drive) {
    addCommands(
        // new PrepareToPickup(claw, wrist, elevator),
        // new Pickup(claw, wrist, elevator),
        // new SetDistanceToBox(0.10, drive),
        new DriveStraight(6, drive), // Use encoders if ultrasonic is broken
        // new Place(claw, wrist, elevator),
        // new SetDistanceToBox(0.60, drive),
        new DriveStraight(4, drive),
        new DriveStraight(3, drive)
        ); // Use Encoders if ultrasonic is broken
        // Commands.parallel(new SetWristSetpoint(-45, wrist), new CloseClaw(claw)));
  }
}