package org.frogforce503.lib.auto.follower;

import java.util.function.Supplier;

import org.frogforce503.lib.swerve.SwerveCommand;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.RotationSequence;
import org.frogforce503.lib.util.AllianceFlipUtil;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.robot2024.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveFollowPathCommand extends Command {

    private final Supplier<PlannedPath> dynamicPath;
    private final Supplier<Boolean> isAiming;
    private final boolean endsStopped;
    private PlannedPath path;
    private final SwervePathFollower controller;
    private final Timer timer;

    private double lastTime = 0;
    private Rotation2d lastAngle = new Rotation2d();
    private Translation2d lastPosition = new Translation2d();

    private Supplier<Rotation2d> headingOverride = null;

    private boolean isFetchingAtEnd = false;
    private double seek = 0;
    private double timeToFetchAtEnd = 0;

    DoubleLogEntry timeLog, xLog, yLog, vxLog, vyLog, desiredXLog, desiredYLog, desiredVxLog, desiredVyLog;

    public SwerveFollowPathCommand(Supplier<PlannedPath> dynamicPath, Supplier<Boolean> isAiming) {
        this.dynamicPath = dynamicPath;
        this.isAiming = isAiming;
        this.controller = new SwervePathFollower(Robot.bot.autoXController, Robot.bot.autoYController, Robot.bot.autoThetaController);
        this.controller.setTolerance(new Pose2d(new Translation2d(Units.inchesToMeters(1.75), Units.inchesToMeters(1.75)), Rotation2d.fromDegrees(1)));
        
        this.timer = new Timer();
        this.endsStopped = this.dynamicPath.get().getDriveTrajectory().sample(this.dynamicPath.get().getTotalTimeSeconds()).velocityMetersPerSecond < 0.1;

        DataLog log = DataLogManager.getLog();
        this.timeLog = new DoubleLogEntry(log, "wheels/timestamp");
        this.xLog = new DoubleLogEntry(log, "wheels/x");
        this.yLog = new DoubleLogEntry(log, "wheels/y");
        this.desiredXLog = new DoubleLogEntry(log, "wheels/xTarget");
        this.desiredYLog = new DoubleLogEntry(log, "wheels/yTarget");
        this.vxLog = new DoubleLogEntry(log, "wheels/vx");
        this.vyLog = new DoubleLogEntry(log, "wheels/vy");
        this.desiredVxLog = new DoubleLogEntry(log, "wheels/vxTarget");
        this.desiredVyLog = new DoubleLogEntry(log, "wheels/vyTarget");

        addRequirements(RobotContainer.drive);
    }

    public SwerveFollowPathCommand(Supplier<PlannedPath> pathS) {
        this(pathS, () -> false);
    }

    public SwerveFollowPathCommand(PlannedPath path, Supplier<Boolean> isAiming) {
        this(() -> path, isAiming);
    }

    public SwerveFollowPathCommand(PlannedPath path, double seek) {
        this(() -> path);
        this.seek = seek;
    }

    public SwerveFollowPathCommand(PlannedPath path) {
        this(path, () -> false);
    }

    public SwerveFollowPathCommand(PlannedPath path, boolean isFetchingAtEnd) {
        this(path);
        this.isFetchingAtEnd = true;
    }

    public SwerveFollowPathCommand fetchAtEndFor(double time) {
        this.timeToFetchAtEnd = time;
        return this;
    }

    public void configureMarkers() { } // TODO: Make this work

    public SwerveFollowPathCommand setHeadingOverride(Supplier<Rotation2d> func) {
        this.headingOverride = func;
        return this;
    }

    @Override
    public void initialize() {       
        this.path = dynamicPath.get(); 
        this.timer.reset();
        this.controller.reset();
        this.timer.start();
        
        DataLogManager.start();

        lastPosition = this.path.getInitialHolonomicPose().getTranslation();
        lastAngle = this.path.getInitialHolonomicPose().getRotation();
        lastTime = 0;

        var poses = this.dynamicPath.get().getDriveTrajectory().getStates().stream()
                .map(state -> (state.poseMeters))
                .toArray(Pose2d[]::new);
        RobotContainer.drive.getField().getObject("CurrentTrajectory").setPoses(poses);
    }

    @Override
    public void execute() {
        this.path = dynamicPath.get();

        double currentTime = this.timer.get() + this.seek;

        boolean timeForFetch = this.path.getTotalTimeSeconds() - currentTime <= timeToFetchAtEnd;
        boolean capableOfFetching = RobotContainer.noteDetector.cameraStatus() && !RobotContainer.feeder.noteInEntrySensor() && RobotContainer.noteDetector.noteInView();
        
        Pose2d currentPose = Drive.getInstance().getPose();

        if (timeForFetch && capableOfFetching && timeToFetchAtEnd > 0) {
            RobotContainer.drive.acceptSwerveCommand(
                RobotContainer.drive.getFetchingCommand()
            );
            RobotContainer.driverfeedback.setToFetch();
        } else {
            PlannedPath.HolonomicState desiredState = this.path.sample(currentTime);

            if (this.isAiming.get()) {
                desiredState.holonomicAngle = ShotPlanner.getInstance().getAimTarget();
            } else {
                desiredState.holonomicAngle = headingOverride != null ? headingOverride.get() : desiredState.holonomicAngle;
            }


            SmartDashboard.putNumber("PathFollower/Desired Robot X", desiredState.poseMeters.getX());
            SmartDashboard.putNumber("PathFollower/Desired Robot Y", desiredState.poseMeters.getY());
            SmartDashboard.putNumber("PathFollower/Desired Robot Theta (deg)", desiredState.holonomicAngle.getDegrees());

            SmartDashboard.putNumber("PathFollower/X Error", currentPose.getX() - desiredState.poseMeters.getX());
            SmartDashboard.putNumber("PathFollower/Y Follower Error", currentPose.getY() - desiredState.poseMeters.getY());
            SmartDashboard.putNumber("PathFollower/Theta Follower Error (deg)", currentPose.getRotation().getDegrees() - desiredState.holonomicAngle.getDegrees());

            this.timeLog.append(currentTime);
            this.desiredXLog.append(desiredState.poseMeters.getX());
            this.desiredYLog.append(desiredState.poseMeters.getY());
            this.xLog.append(currentPose.getX());
            this.yLog.append(currentPose.getY());
            
            ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);

            // System.out.println(targetChassisSpeeds);

            SmartDashboard.putNumber("Desired X Velocity", targetChassisSpeeds.vxMetersPerSecond);
            
            this.desiredVxLog.append(targetChassisSpeeds.vxMetersPerSecond);
            this.desiredVyLog.append(targetChassisSpeeds.vyMetersPerSecond);

            Translation2d measuredVelocity = currentPose.getTranslation().minus(lastPosition).times(1/(currentTime-lastTime));
            SmartDashboard.putNumber("Measured X Velocity", measuredVelocity.getX());

            this.vxLog.append(measuredVelocity.getX());
            this.vyLog.append(measuredVelocity.getY());

            RobotContainer.drive.acceptSwerveCommand(
                new SwerveCommand.RobotCentric()
                .withVelocityX(targetChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(targetChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(targetChassisSpeeds.omegaRadiansPerSecond)
            );        // SmartDashboard.putNumber("PPSwerveControllerCommand_xOutput", targetChassisSpeeds.vxMetersPerSecond);
            lastAngle = desiredState.holonomicAngle;  
        }    // SwerveModuleState[] targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

        // this.outputModuleStates.accept(targetModuleStates);
        lastTime = currentTime;
        lastPosition = currentPose.getTranslation();
    }

    @Override
    public boolean isFinished() {
        boolean timeHasFinished = this.timer.hasElapsed(this.path.getTotalTimeSeconds());
        boolean poseTolerance = this.controller.atReference();
        boolean tooLong = timeHasFinished && this.timer.hasElapsed(this.path.getTotalTimeSeconds() + 0.5);

        boolean m_isFinished = (timeHasFinished && poseTolerance) || tooLong || (timeToFetchAtEnd > 0 && RobotContainer.feeder.noteInEntrySensor());
        SmartDashboard.putNumber("Path Has Finished", m_isFinished ? 1.0 : 0);

        return timeHasFinished;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDD " + interrupted);
        Logger.recordOutput(
            "Swerve/CurrentTrajectory", new Pose2d[] {});
        RobotContainer.drive.getField().getObject("CurrentTrajectory").setPoses();

        if (this.endsStopped && !this.isFetchingAtEnd) {
            RobotContainer.drive.acceptSwerveCommand(new SwerveCommand.SwerveDriveBrake());
        }

        if (!RobotContainer.feeder.noteInExitSensor() && timeToFetchAtEnd > 0) {
            RobotContainer.driverfeedback.setToDefault();
        }
    }
}