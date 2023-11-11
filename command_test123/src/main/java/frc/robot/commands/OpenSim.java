package frc.robot.commands;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.function.DoubleSupplier;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class OpenSim extends CommandBase {
    private final int port;
    private final Drivetrain drive;
    private ServerSocket serverSocket;
    private Socket unitySocket;

    public OpenSim(int _port, Socket _socket, ServerSocket _serverSocket, Drivetrain _drive) {
        port = _port;
        unitySocket = _socket;
        serverSocket = _serverSocket;
        drive = _drive;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        System.out.println("Opening the simulator");
        try {
            serverSocket = new ServerSocket(port);
            System.out.println("Waiting for Unity connection on port " + port);
            unitySocket = serverSocket.accept();
            System.out.println("Unity connected!");

            // Start a thread for sending data to Unity
            Thread sendToUnityThread = new Thread(() -> {
                try {
                    PrintWriter out = new PrintWriter(unitySocket.getOutputStream(), true);
                    BufferedReader in = new BufferedReader(new InputStreamReader(unitySocket.getInputStream()));

                    // Send data to Unity
                    while (true) {
                        String dataToSend = "" +
                                "clawLeft:" + SmartDashboard.getNumber("Claw - Left", 0) +
                                "/elevatorRight:" + SmartDashboard.getNumber("Elevator - Right", 0) +
                                "/drivetrainLeft:" + drive.m_leftMotor.get() +
                                "/drivetrainRight:" + drive.m_rightMotor.get();

                        out.println(dataToSend);
                        Thread.sleep(20); // Adjust the interval as needed
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
        ;
    }
}
