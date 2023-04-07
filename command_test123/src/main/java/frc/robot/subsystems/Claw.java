package frc.robot.subsystems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The claw subsystem is a simple system with a motor for opening and closing. If using stronger
 * motors, you should probably use a sensor so that the motors don't stall.
 */
public class Claw extends SubsystemBase {
  //private final TalonSRX m_motorRight = new TalonSRX(8);
  private final TalonSRX m_motorLeft = new TalonSRX(1);

  TalonSRXSimCollection m_leftSim = m_motorLeft.getSimCollection();
  //TalonSRXSimCollection m_rightSim = m_motorRight.getSimCollection();
  //private final CAN m_motorLeft = new CAN(0);
  
  private final DigitalInput m_contact = new DigitalInput(7);
  // private final double m_speed = 0.085;
  private final double m_speed = 0.90;

  /** Create a new claw subsystem. */
  public Claw() {
    HAL.initialize(200, 0);
    Robot.initTalon(m_motorLeft);
    //Robot.initTalon(m_motorRight);
  }


  public void log() {
    //SmartDashboard.putData("Claw switch", m_contact);
    SmartDashboard.putNumber("Claw - Left", m_leftSim.getMotorOutputLeadVoltage());
    //SmartDashboard.putNumber("Claw - Right", m_motorRight.getMotorOutputPercent());
    Drivetrain.m_driveSim.update(0.020);
  }

  /** Set the claw motor to move in the open direction. */
  public void open() {
    m_motorLeft.set(ControlMode.PercentOutput, -m_speed);
    //m_motorRight.set(ControlMode.PercentOutput, m_speed);
  }

  /** Set the claw motor to move in the close direction. */
  public void close() {
    m_motorLeft.set(ControlMode.PercentOutput, m_speed);
    //m_motorRight.set(ControlMode.PercentOutput, -m_speed);
  }

  /** Stops the claw motor from moving. */
  public void stop() {
    m_motorLeft.set(ControlMode.PercentOutput, 0);
    //m_motorRight.set(ControlMode.PercentOutput, 0);
  }

  /** Return true when the robot is grabbing an object hard enough to trigger the limit switch. */
  public boolean isGrabbing() {
    return m_contact.get();
  }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();
  }

  @Override
  public void simulationPeriodic() {
    /* Pass the robot battery voltage to the simulated Talon FXs */
    HAL.initialize(200, 0);
    m_leftSim.setBusVoltage(RobotController.getBatteryVoltage());
    //m_rightSim.setBusVoltage(RobotController.getBatteryVoltage());

    Drivetrain.m_driveSim.update(0.020);
  }
}
