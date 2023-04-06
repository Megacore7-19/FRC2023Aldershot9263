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
  private final double distMult = 0.7;
  public Autonomous(Drivetrain drive) {
    /*    Bottom Track | Autonomous V1    */
    addCommands(
      //AUTO - DRIVE FORWARDS - NO BUMP - NOT USED
      // new TankDriveStraight(-1 * distMult, drive, 1.75)

      //AUTO - CHARGE STATION - NOT USED
      //new TankDriveStraight(-1 * distMult, drive, 3) //time could be extended, need to add brake mode to the motors so that they don't fall off
      
      //AUTO - DRIVE FORWARD - BUMP - NOT USED
      //new TankDriveStraight(-1 * distMult, drive, 1)


      /*
      USED AUTO CODE      ||
                          ||
                          \/
      */

      //AUTO - DRIVE FORWARD - DROP CUBE - SHORT SIDE
      // new TankDriveStraight(1 * distMult, drive, 0.4),
      // new TankDriveStraight(-1 * distMult, drive, 1.15),
      // new TankDriveStraight(1 * distMult, drive, 2.8)

      // AUTO - DRIVE FORWARD - DROP CUBE - LONG SIDE
      new TankDriveStraight(1 * distMult, drive, 0.4),
      new TankDriveStraight(-1 * distMult, drive, 1.15),
      new TankDriveStraight(1 * distMult, drive, 4)

      //AUTO - CHARGE STATION 
      // new TankDriveStraight(1 * distMult, drive, 3.7) 
    
      //AUTO - CHARGE STATION - OVER AND BACK
      // new TankDriveStraight(1 * distMult, drive, 5.5),
      // new TankDriveStraight(0 * distMult, drive, 0.75),
      // new TankDriveStraight(-1 * distMult, drive, 4)

      //AUTO - CHARGE STATION - DROP CUBE
      // new TankDriveStraight(1 * distMult, drive, 0.4),
      // new TankDriveStraight(0 * distMult, drive, 0.75),
      // new TankDriveStraight(-1 * distMult, drive, 1.15),
      // new TankDriveStraight(0 * distMult, drive, 0.75),
      // new TankDriveStraight(1 * distMult, drive, 5)

      //AUTO - CHARGE STATION - DROP CUBE + OVER AND BACK (MAYBE TOO UN RELIABLE)
      // new TankDriveStraight(1 * distMult, drive, 0.4),
      // new TankDriveStraight(0 * distMult, drive, 0.75),
      // new TankDriveStraight(-1 * distMult, drive, 1.15),
      // new TankDriveStraight(0 * distMult, drive, 0.75),
      // new TankDriveStraight(1 * distMult, drive, 7),
      // new TankDriveStraight(0 * distMult, drive, 0.75),
      // new TankDriveStraight(-1 * distMult, drive, 3.7)


        );
    




    /*    Middle Track W/ Ramp | Autonomous V2    */
    addCommands(
      // new TankDriveStraight(-1 * distMult, drive, 0.75),
      new TankDriveStraight(0, drive, 0.25)
      // new TankDriveStraight(1 * distMult, drive, 2.864),
      // new TankDriveStraight(1.25 * distMult, drive, 0.764),
      // new TankDriveStraight(0.933 * distMult, drive, 1.35)
        );

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