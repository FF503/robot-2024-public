package org.frogforce503.robot2024.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;

public class PigeonGyro extends BaseGyro {

    PigeonIMU imu;

    public PigeonGyro(int deviceNumber) {
        imu = new PigeonIMU(deviceNumber);
    }

    @Override
    public void initialize() {
        imu.configFactoryDefault();
    }

    @Override
    public void setYaw(Rotation2d angle) {
        imu.setYaw(angle.getDegrees());
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(imu.getYaw());
    }

    /*
    @Override
    public void setPitch(Rotation2d angle) {
        // TODO Auto-generated method stub
    }
    */

    @Override
    public Rotation2d getPitch() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(imu.getPitch());
    }

    @Override
    public Rotation2d getRoll() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(imu.getRoll());
    }
}