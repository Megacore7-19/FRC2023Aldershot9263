package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The claw subsystem is a simple system with a motor for opening and closing. If using stronger
 * motors, you should probably use a sensor so that the motors don't stall.
 */
public class ElevatorMain extends SubsystemBase {
  private final PWMSparkMax m_motorRight = new PWMSparkMax(5);
  
//  private final PWMSparkMax m_motorLeft = new PWMSparkMax(6);
  //private final DigitalInput m_contact = new DigitalInput(5);
  //private final double m_speedL = 0.15;
  private final double m_speedR = 0.50;
  /** Create a new claw subsystem. */
  public ElevatorMain() {
    // Let's name everything on the LiveWindow
    addChild("MotorLeft", m_motorRight);
    addChild("MotorRight", m_motorRight);
  }

  public void log() {
    //SmartDashboard.putData("Claw switch", m_contact);
    //SmartDashboard.putNumber("Elevator - Left", m_motorLeft.get());
    SmartDashboard.putNumber("Elevator - Right", m_motorRight.get());
  }

  /** Set the claw motor to move in the open direction. */
  public void open(double degree) {
    //m_motorLeft.set(-m_speedL * degree);
    m_motorRight.set(m_speedR * degree);
  }

  /** Set the claw motor to move in the close direction. */
  public void close(double degree) {
   // m_motorLeft.set(m_speedL * degree);
    m_motorRight.set(-m_speedR * degree);
  }

  /** Stops the claw motor from moving. */
  public void stop() {
    //m_motorLeft.set(0);
    m_motorRight.set(0);
  }

  /** Return true when the robot is grabbing an object hard enough to trigger the limit switch. */
//   public boolean isGrabbing() {
//     //return m_contact.get();
//   }

  /** Call log method every loop. */
  @Override
  public void periodic() {
    log();

    // if (m_joystick.getY() > 0.1) {
    //   open(Math.abs(m_joystick.getY()));
    // } else if (m_joystick.getY() < 0.1) {
    //   close(Math.abs(m_joystick.getY()));
    // }
  }
}