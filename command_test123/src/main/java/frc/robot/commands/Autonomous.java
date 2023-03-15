// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.TankDriveStraight;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomous extends SequentialCommandGroup {
  // distMult modifies how fast the motors go.
  // The formula to calculate the motor speed in percentage is :  
  // '  (distMult * 100) * distMult '
  // if distMult = 0.5, motorSpeed = 25%
  // if distMult = 0.6, motorSpeed = 36%
  // if distMult = 0.8, motorSpeed = 64%;
  // At MotorSpeed 25%, the robot moves 25in. per second
  private final double distMult = 0.6;
  public Autonomous(Drivetrain drive) {
    /*    Bottom Track | Autonomous V1    */
    // addCommands(
    //   new TankDriveStraight(-1 * distMult, drive, 0.75),
    //   new TankDriveStraight(0, drive, 0.25),
    //   new TankDriveStraight(1.075 * distMult, drive, 8),
    //   new TankDriveStraight(0, drive, 0.25),
    //   new TankDriveStraight(-1 * distMult, drive, 3)
    //     );
    
    /*    Middle Track W/ Ramp | Autonomous V2    */
    // addCommands(
    //   new TankDriveStraight(-1 * distMult, drive, 0.75),
    //   new TankDriveStraight(0, drive, 0.25),
    //   new TankDriveStraight(1 * distMult, drive, 2.864),
    //   new TankDriveStraight(1.25 * distMult, drive, 0.764),
    //   new TankDriveStraight(0.933 * distMult, drive, 1.35)
    //     );

    /*    Top Track | Autonomous V3    */
    // addCommands(
    //   new TankDriveStraight(-1 * distMult, drive, 0.75),
    //   new TankDriveStraight(0, drive, 0.25),
    //   new TankDriveStraight(1 * distMult, drive, 5.530),
    //   new TankDriveStraight(0, drive, 0.25),
    //   new TankDriveStraight(-1 * distMult, drive, 2.762)
    // );
  }
}