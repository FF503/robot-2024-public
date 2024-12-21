package org.frogforce503.robot2024.subsystems.drive.drivetrain;

import java.util.function.Consumer;

import org.frogforce503.lib.swerve.SwerveCommand;
import org.frogforce503.lib.swerve.SwerveDriveState;
import org.frogforce503.robot2024.Robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimDrive implements BaseDrivetrain {

    private Pose2d pose = new Pose2d();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] states = new SwerveModuleState[] { new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    private SwerveDriveKinematics kinematics;

    private double lastUpdate = -1.0;
    private double dt = 0;

    private Consumer<SwerveDriveState> telemetryMethod;

    public SimDrive() {
        kinematics = Robot.bot.kinematics;
    }

    @Override
    public void drive(SwerveCommand request) {
        this.states = request.getDriven(getDefaultParameters());
        this.chassisSpeeds = kinematics.toChassisSpeeds(this.states);
    }

    @Override
    public Pose2d getPoseMeters() {
        return this.pose;
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return this.pose.getRotation();
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public void setAngle(Rotation2d angle) {
        this.pose = new Pose2d(this.pose.getTranslation(), angle);
    }


    @Override
    public void periodic() {
        double t = Timer.getFPGATimestamp();
        if (lastUpdate > 0) {
            dt = t - lastUpdate;
            pose = pose.exp(
                new Twist2d(
                    chassisSpeeds.vxMetersPerSecond * dt, 
                    chassisSpeeds.vyMetersPerSecond * dt, 
                    chassisSpeeds.omegaRadiansPerSecond * dt
                )
            );
        }
        lastUpdate = t;

    }

    @Override
    public void stop() {
        this.chassisSpeeds = new ChassisSpeeds();
    }

    @Override
    public SwerveDriveState getCurrentState() {
        SwerveDriveState currentState = new SwerveDriveState();
        currentState.SuccessfulDaqs = 0;
        currentState.FailedDaqs = 0;
        currentState.Pose = getPoseMeters();
        currentState.ModuleStates = this.states;
        currentState.OdometryPeriod = 0.02;
        return currentState;
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return chassisSpeeds;
    }

    @Override
    public void acceptVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> stdDevs) {
        
    }
}
