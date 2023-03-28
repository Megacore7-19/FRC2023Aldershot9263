package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Teleop Camera subsystem
 */
public class TeleopCamera extends SubsystemBase {

    public UsbCamera frontCamera;

    public TeleopCamera()
    {
        // // start capture from the camera at the front plugged into USB slot 0
        // // frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
        // frontCamera = CameraServer.startAutomaticCapture(0);
        // // set the stream's resolution to 320x240
        // frontCamera.setResolution(320, 240);
        // // set the stream's frames per second to 15
        // frontCamera.setFPS(15);
    }

    public void initDefaultCommand()
    {
    }
}