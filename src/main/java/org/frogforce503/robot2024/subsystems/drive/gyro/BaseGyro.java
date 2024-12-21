package org.frogforce503.robot2024.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class BaseGyro {
    public abstract void initialize();

    public abstract void setYaw(Rotation2d angle); // calls hardware method to set angle
    public abstract Rotation2d getYaw();

    // public abstract void setPitch(Rotation2d angle); // calls hardware method to set angle
    public abstract Rotation2d getPitch();

    public abstract Rotation2d getRoll();
}
