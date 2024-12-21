package org.frogforce503.robot2024.subsystems.drive.drivetrain;

import java.util.function.Consumer;

import org.frogforce503.lib.swerve.SwerveCommand;
import org.frogforce503.lib.swerve.SwerveDriveState;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.hardware.tunerconstants.TunerConstantsPracticeBot;
import org.frogforce503.robot2024.subsystems.drive.SwerveModuleLoader;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;

public class PhoenixSwerve extends SwerveDrivetrain implements BaseDrivetrain {

    private Rotation2d zeroOffset = new Rotation2d();
    private Transform2d poseOffset = new Transform2d();
     
    public PhoenixSwerve() {
        super(Robot.bot.phoenixConstants, SwerveModuleLoader.getInstance().loadPhoenixModules());
        // m_odometry = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(), m_modulePositions, new Pose2d(),
        //     VecBuilder.fill(0.1, 0.1, Units),
        //     VecBuilder.fill(0.9, 0.9, 0.9)
        // );
    }

    @Override
    public void drive(SwerveCommand request) {
        this.setControl(request);        
    }

    @Override
    public Pose2d getPoseMeters() {
        return this.getState().Pose;//this.getState().Pose.rotateBy(zeroOffset.unaryMinus().plus(new Rotation2d(Math.PI))).plus(poseOffset);
    }

    @Override
    public Rotation2d getAngleRotation2d() {
        return this.getPoseMeters().getRotation();
    }

    @Override
    public void setPose(Pose2d pose) {
        System.out.println("Setting pose to " + pose);
        seedFieldRelative(pose);
        System.out.println("Set pose to " + getPoseMeters());
        // this.tareEverything();
        // this.poseOffset = new Transform2d(pose, new Pose2d()).inverse();
        // this.zeroOffset = new Rotation2d();

        // System.out.println(getPoseMeters().toString());
    }

    @Override
    public void setAngle(Rotation2d angle) {
        setPose(new Pose2d(getPoseMeters().getTranslation(), angle));
        // this.zeroOffset = this.getPoseMeters().getRotation().minus(angle);
    }

    @Override
    public void resetGyro() {
        setAngle(new Rotation2d());
    }

    @Override
    public void acceptVisionMeasurement(Pose2d poseEstimate, double timestamp, Matrix<N3, N1> stdDevs) {
        if (stdDevs == null) {
            this.addVisionMeasurement(poseEstimate, timestamp);
        } else {
            this.addVisionMeasurement(poseEstimate, timestamp, stdDevs);
        }
    }

    @Override
    public void stop() {
        this.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(m_kinematics.toChassisSpeeds(getState().ModuleStates), getAngleRotation2d());
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation())
            updateSimState(0.02, 12);
    }

    public org.frogforce503.lib.swerve.SwerveDriveState getCurrentState() {
        return org.frogforce503.lib.swerve.SwerveDriveState.translate(getState());
    }
}
