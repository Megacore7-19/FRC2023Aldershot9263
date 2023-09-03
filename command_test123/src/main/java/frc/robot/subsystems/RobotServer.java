package frc.robot.subsystems;

import java.io.*;
import java.net.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotServer extends SubsystemBase {
    private int port;
    private ServerSocket serverSocket;

    public RobotServer(int port) {
        this.port = port;
    }

    public void startServer() {
        try {
            serverSocket = new ServerSocket(port);
            System.out.println("RobotServer listening on port " + port);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void runServer() {
        System.out.println("Hola");
        try (Socket clientSocket = serverSocket.accept()) {
            OutputStream outStream = clientSocket.getOutputStream();
            PrintWriter out = new PrintWriter(outStream, true);

            // Send data to Unity or other clients
            String dataToSend = "Hello, Unity!";
            out.println(dataToSend);

            clientSocket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void stopServer() {
        try {
            if (serverSocket != null && !serverSocket.isClosed()) {
                serverSocket.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}