package org.frogforce503.robot2024.subsystems.drive.drivetrain;

import java.util.function.Consumer;

import org.frogforce503.lib.swerve.SwerveCommand;
import org.frogforce503.lib.swerve.SwerveDriveState;
import org.frogforce503.robot2024.Robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

// abstracts the core functionality
public interface BaseDrivetrain {
    public abstract void drive(SwerveCommand request);
    public abstract Pose2d getPoseMeters();
    public abstract ChassisSpeeds getVelocity();
    public abstract Rotation2d getAngleRotation2d();
    public abstract void setPose(Pose2d pose);
    public abstract void setAngle(Rotation2d angle);
    public default void resetGyro() {
        setAngle(new Rotation2d());
    }
    public void acceptVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> stdDevs);
    public abstract void stop();
    public default void periodic() {};
    public abstract SwerveDriveState getCurrentState();
    
    public default SwerveControlRequestParameters getDefaultParameters() {
        SwerveControlRequestParameters parameters = new SwerveControlRequestParameters();
        parameters.currentPose = getPoseMeters();
        parameters.swervePositions = Robot.bot.kModulePositions;
        parameters.kinematics = Robot.bot.kinematics;
        parameters.updatePeriod = 0.02;
        parameters.timestamp = Timer.getFPGATimestamp();
        return parameters;
    }
}
