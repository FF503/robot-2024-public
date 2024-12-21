package org.frogforce503.robot2024.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Gyro extends BaseGyro {

    Pigeon2 imu;

    public Pigeon2Gyro (int deviceNumber, String canbus) {
        imu = new Pigeon2(deviceNumber, canbus);
    }

    public Pigeon2Gyro (int deviceNumber) {
        imu = new Pigeon2(deviceNumber);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        imu.configFactoryDefault();
    }

    @Override
    public void setYaw(Rotation2d angle) {
        // TODO Auto-generated method stub
        imu.setYaw(angle.getDegrees());
    }

    @Override
    public Rotation2d getYaw() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(imu.getYaw());
    }

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