package org.frogforce503.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseSender {
    public static void send(String key, Pose2d... value) {
        double[] data = new double[value.length * 3];
        for (int i = 0; i < value.length; i++) {
          data[i * 3] = value[i].getX();
          data[i * 3 + 1] = value[i].getY();
          data[i * 3 + 2] = value[i].getRotation().getRadians();
        }
        SmartDashboard.putNumberArray(key, data);
    }

    public static void send(String key, Pose3d... value) {
        double[] data = new double[value.length * 7];
        for (int i = 0; i < value.length; i++) {
            data[i * 7] = value[i].getX();
            data[i * 7 + 1] = value[i].getY();
            data[i * 7 + 2] = value[i].getZ();
            data[i * 7 + 3] = value[i].getRotation().getQuaternion().getW();
            data[i * 7 + 4] = value[i].getRotation().getQuaternion().getX();
            data[i * 7 + 5] = value[i].getRotation().getQuaternion().getY();
            data[i * 7 + 6] = value[i].getRotation().getQuaternion().getZ();
        }
        SmartDashboard.putNumberArray(key, data);
    }
}
