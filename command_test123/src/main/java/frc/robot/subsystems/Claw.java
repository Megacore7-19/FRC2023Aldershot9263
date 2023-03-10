package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The claw subsystem is a simple system with a motor for opening and closing. If using stronger
 * motors, you should probably use a sensor so that the motors don't stall.
 */
public class Claw extends SubsystemBase {
  private final PWMSparkMax m_motorRight = new PWMSparkMax(7);
  private final PWMSparkMax m_motorLeft = new PWMSparkMax(8);
  private final DigitalInput m_contact = new DigitalInput(5);
  private final double m_speed = 0.5;

  /** Create a new claw subsystem. */
  public Claw() {
    // Let's name everything on the LiveWindow
    addChild("MotorLeft", m_motorRight);
    addChild("MotorRight", m_motorRight);
    //addChild("Limit Switch", m_contact);
  }

  public void log() {
    SmartDashboard.putData("Claw switch", m_contact);
  }

  /** Set the claw motor to move in the open direction. */
  public void open() {
    m_motorLeft.set(-m_speed);
    m_motorRight.set(m_speed);
  }

  /** Set the claw motor to move in the close direction. */
  public void close() {
    m_motorLeft.set(m_speed);
    m_motorRight.set(-m_speed);
  }

  /** Stops the claw motor from moving. */
  public void stop() {
    m_motorLeft.set(0);
    m_motorRight.set(0);
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
}
