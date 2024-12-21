package org.frogforce503.robot2024.subsystems.sim;

import org.frogforce503.lib.util.PoseSender;
import org.frogforce503.robot2024.subsystems.drive.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimGamePiece {
    private static SimGamePiece instance;
    
    private static Pose3d fieldPosition = new Pose3d();
    private static boolean beenPickedUp = false;

    private static Rotation3d coneRotation = new Rotation3d(0.0, 0.0, 0.0);

    public static SimGamePiece getInstance() {
        if (instance == null) { instance = new SimGamePiece(); }
        return instance;
    }

    public SimGamePiece() {}

    private static double distanceFromObject(Pose2d pose) {
        return Math.hypot(Math.abs(fieldPosition.getX() - pose.getX()), Math.abs(fieldPosition.getY() - pose.getY()));  
    }

    public static void reset() {
        fieldPosition = new Pose3d(7.5, 3.0, 0.0, coneRotation);
    }

    private static void pickedUp() {
        fieldPosition = new Pose3d(100, 3.0, 0.0, coneRotation);
    }

    public void update() {
        if (distanceFromObject(Drive.getInstance().getPose()) > 0 && distanceFromObject(Drive.getInstance().getPose()) < 0.5) { // had to do between 0 & 0.5 from 2 cones in sim (only publishing one pose but 2 objects appearing)
            beenPickedUp = true;
            pickedUp();
        }

        SmartDashboard.putNumber("distfromobj", distanceFromObject(Drive.getInstance().getPose()));

        if (!beenPickedUp) {
            fieldPosition = new Pose3d(7.5, 3.0, 0.0, coneRotation);
        }

        PoseSender.send("BlueCone", fieldPosition);
    }
}
