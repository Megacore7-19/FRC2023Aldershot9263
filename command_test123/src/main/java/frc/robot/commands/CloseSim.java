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

public class CloseSim extends CommandBase {
    private final int port;
    private ServerSocket serverSocket;
    private Socket unitySocket;

    public CloseSim(int _port, Socket _socket, ServerSocket _serverSocket) {
        port = _port;
        unitySocket = _socket;
        serverSocket = _serverSocket;
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
        System.out.println("Closing the simulator");
        try {
            if (unitySocket != null) {
                unitySocket.close();
            }
            if (serverSocket != null && !serverSocket.isClosed()) {
                serverSocket.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
