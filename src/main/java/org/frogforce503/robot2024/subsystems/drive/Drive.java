package org.frogforce503.robot2024.subsystems.drive;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.frogforce503.lib.swerve.SwerveCommand;
import org.frogforce503.lib.util.Logic;
import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.RobotStatus;
import org.frogforce503.robot2024.RobotStatus.AllianceColor;
import org.frogforce503.robot2024.hardware.RobotHardware;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.robot2024.subsystems.drive.drivetrain.BaseDrivetrain;
import org.frogforce503.robot2024.subsystems.drive.drivetrain.PhoenixSwerve;
import org.frogforce503.robot2024.subsystems.drive.drivetrain.SimDrive;
import org.frogforce503.robot2024.subsystems.drive.drivetrain.SwerveDrive;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

    private static Drive mInstance = null;
    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    // hardware
    private BaseDrivetrain swerve;
    private DrivetrainType type = RobotHardware.getInstance().drivetrainType;

    // constants
    public static final double SLOW_TRANSLATION_METERS_PER_SECOND = 1.5;
    public static final double FAST_TRANSLATION_METERS_PER_SECOND = Robot.bot.frontLeftConstants.SpeedAt12VoltsMps;
    public static final double FAST_ROTATION_RADIANS_PER_SECOND = Units.degreesToRadians(360); 
    public static final double SUPER_FAST_ROTATION_RADIANS_PER_SECOND = FAST_ROTATION_RADIANS_PER_SECOND * 2.25; 

    public static final double SLOW_ROTATION_RADIANS_PER_SECOND = Units.degreesToRadians(50);

    public static final double MAX_ACCELERATION_METERS_PER_SEC_PER_SEC = FAST_TRANSLATION_METERS_PER_SECOND * 1.2;
    public static final double MAX_ANGULAR_ACCLERATION_RAD_PER_SEC_PER_SEC = 5 * Math.PI / 2.0;

    public static final double JOYSTICK_DRIVE_TOLERANCE = 0.2;
    public static final double JOYSTICK_TURN_TOLERANCE = 0.2;

    private static final String[] moduleNames = { "FrontLeft", "FrontRight", "BackLeft", "BackRight" };

    private Transform2d currentVelocity = new Transform2d();
    private Pose2d currentPose = new Pose2d();
    private Pose2d lastPose = new Pose2d();
    private static Translation2d lastJoystickCommand;

    // teleop
    private boolean aimingTeleop = false;
    private boolean slowmodeTeleop = false;
    private boolean tryingToDriveTeleop = false;
    private boolean onTargetTeleop = false;
    private boolean wasStabilizing = false;

    private Debouncer stabilizationDebouncer = new Debouncer(0.2);
    private Rotation2d stabilizationHeading = new Rotation2d();

    private PIDController pickupController = new PIDController(1, 0, 0);
    private PIDController dopController = new PIDController(0.5, 0, 0); // dop stands for drive-to-point
    private PIDController followAprilTagController = new PIDController(0.25, 0, 0);

    private double dt = 0.02;
    private double t0 = 0;

    private Field2d field = new Field2d();
    private SwerveVisualizer visualizer = new SwerveVisualizer(FAST_TRANSLATION_METERS_PER_SECOND);

    private SwerveCommand currentRequest;
    
    /*
     * Constructor 
     */
    private Drive() {
        if (RobotBase.isSimulation()) {
            this.type = DrivetrainType.SIM_SWERVE;
        }

        switch (this.type) {
            case SIM_SWERVE: {
                this.swerve = new SimDrive();
                break;
            }
            case PHOENIX_SWERVE: {
                this.swerve = new PhoenixSwerve();
                break;
            }
            case SWERVE: {
                this.swerve = new SwerveDrive();
                break;
            }
        }

        SmartDashboard.putData("Field", field);
        visualizer.initialize();

        dopController.setTolerance(0.125); // can change
    }

    public void acceptSwerveCommand(SwerveCommand commandToAccept) {
        swerve.drive(commandToAccept);
        this.currentRequest = commandToAccept;
    }

    @Override
    public void periodic() {
        if (t0 != 0)
            dt = Timer.getFPGATimestamp() - t0;
        
        this.swerve.periodic();

        this.currentPose = this.swerve.getPoseMeters();
        
        this.calculateVelocity();
        this.outputTelemetry();

        t0 = Timer.getFPGATimestamp();
        this.lastPose = this.currentPose;
    }

    private void calculateVelocity() {
        ChassisSpeeds velocity = this.swerve.getVelocity();
        this.currentVelocity = new Transform2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, new Rotation2d(velocity.omegaRadiansPerSecond));//this.currentPose.minus(this.lastPose).div(dt);
    }

    private void outputTelemetry() {
        Logger.recordOutput("Swerve/Info/DrivetrainType", this.type.name());

        Logger.recordOutput("Swerve/Pose/X", getPose().getX());
        Logger.recordOutput("Swerve/Pose/Y", getPose().getY());
        Logger.recordOutput("Swerve/Pose/Rotation/Yaw", getAngle().getDegrees());
        Logger.recordOutput("Swerve/Velocity/X", this.currentVelocity.getX());
        Logger.recordOutput("Swerve/Velocity/Y", this.currentVelocity.getY());
        Logger.recordOutput("Swerve/Velocity/Rot", this.currentVelocity.getRotation().getDegrees());
        Logger.recordOutput("Swerve/Velocity/Mag", this.currentVelocity.getTranslation().getNorm());

        Logger.recordOutput("Swerve/AttainedWheelSpeed", Units.metersToFeet(this.swerve.getCurrentState().ModuleStates[0].speedMetersPerSecond));

        if (this.currentRequest != null)
            Logger.recordOutput("Swerve/Info/CurrentRequest", this.currentRequest.requestInfo());

        SwerveModuleState[] states = this.swerve.getCurrentState().ModuleStates;
        // Logger.recordOutput("Swerve/ModuleStates", states);

        int i = 0;
        for (SwerveModuleState state : states) {
            Logger.recordOutput("Swerve/Module/" + moduleNames[i] + "/Angle", state.angle.getDegrees());
            Logger.recordOutput("Swerve/Module/" + moduleNames[i] + "/Velocity", state.speedMetersPerSecond);
            i++;
        }

        visualizer.update(states, getAngle());

        displayRobotPose();
    }

    private void displayRobotPose() {
        Translation2d fl = Robot.bot.kVehicleToFrontLeft.rotateBy(getAngle()).plus(getPose().getTranslation());
        Translation2d fr = Robot.bot.kVehicleToFrontRight.rotateBy(getAngle()).plus(getPose().getTranslation());
        Translation2d bl = Robot.bot.kVehicleToBackLeft.rotateBy(getAngle()).plus(getPose().getTranslation());
        Translation2d br = Robot.bot.kVehicleToBackRight.rotateBy(getAngle()).plus(getPose().getTranslation());

        Rotation3d angle = new Rotation3d(0, -Math.PI/2, getAngle().getRadians());

        Logger.recordOutput("Swerve/ModulePoses", new Pose3d[] {new Pose3d(fl.getX(), fl.getY(), 0.5, angle), new Pose3d(fr.getX(), fr.getY(), 0.5, angle), new Pose3d(bl.getX(), bl.getY(), 0.5, angle), new Pose3d(br.getX(), br.getY(), 0.5, angle)});

        Translation2d upperLeft = Robot.bot.kVehicleToFrontLeft.plus(new Translation2d(Units.inchesToMeters(10), 0)).rotateBy(getAngle()).plus(getPose().getTranslation());
        Translation2d upperRight = Robot.bot.kVehicleToFrontRight.plus(new Translation2d(Units.inchesToMeters(10), 0)).rotateBy(getAngle()).plus(getPose().getTranslation());

        Logger.recordOutput("Swerve/CornerPoses", new Pose3d[] {new Pose3d(upperLeft.getX(), upperLeft.getY(), 0.5, angle), new Pose3d(upperRight.getX(), upperRight.getY(), 0.5, angle)});
        
        field.setRobotPose(getPose());
    }

    public void setPose(Pose2d pose) {
        swerve.setPose(pose);
    }

    public void setAngle(Rotation2d rotation) {
        swerve.setAngle(rotation);
    }

    public void seedFieldRelative() {
        System.out.println("SEEDING FIELD RELATIVE POSITION");
        
        if (RobotStatus.getInstance().getAllianceColor() == AllianceColor.BLUE) {
            swerve.resetGyro();
        } else {
            setAngle(Rotation2d.fromDegrees(180));
        }
    }

    public Pose2d getPose() {
        return this.currentPose;
    }

    public Transform2d getVelocity() {
        return this.currentVelocity;
    }

    public Rotation2d getAngle() {
        return this.currentPose.getRotation();
    }

    public Field2d getField() {
        return field;
    }

    private Pair<Translation2d, Double> getTeleopInput() {
        double tSpeed = slowmodeTeleop ? Drive.SLOW_TRANSLATION_METERS_PER_SECOND : Drive.FAST_TRANSLATION_METERS_PER_SECOND;
        double rSpeed = slowmodeTeleop ? Drive.SLOW_ROTATION_RADIANS_PER_SECOND : (RobotContainer.isDriverRightStickPressed.getAsBoolean() ? Drive.SUPER_FAST_ROTATION_RADIANS_PER_SECOND : Drive.FAST_ROTATION_RADIANS_PER_SECOND);

        double xDesired = Util.powerCopySign(Util.handleDeadband(-RobotContainer.driver.getLeftY(), Drive.JOYSTICK_DRIVE_TOLERANCE), 2);
        double yDesired = Util.powerCopySign(Util.handleDeadband(-RobotContainer.driver.getLeftX(), Drive.JOYSTICK_DRIVE_TOLERANCE), 2);
        double rotDesired = Util.powerCopySign(Util.handleDeadband(-RobotContainer.driver.getRightX(), Drive.JOYSTICK_TURN_TOLERANCE), 2);

        double xVel = xDesired * tSpeed;
        double yVel = yDesired * tSpeed;
        double angularVel = rotDesired * rSpeed;
        // TODO: put back when drive gears work

        boolean reversed = RobotStatus.getInstance().getAllianceColor() == AllianceColor.RED;

        Translation2d vel = new Translation2d(xVel, yVel).times(reversed ? -1 : 1);
        // TODO: put back when drive gears work

        Logger.recordOutput("Swerve/DesiredTeleopVelocity/X", xVel);
        Logger.recordOutput("Swerve/DesiredTeleopVelocity/Y", yVel);

        return Pair.of(vel, angularVel);
    }

    public Command driveWithJoysticks() {
        return Commands.run(() -> {
            Pair<Translation2d, Double> input = getTeleopInput();
            Translation2d translation = input.getFirst();
            double omega = input.getSecond().doubleValue();
            // double tSpeed = slowmodeTeleop ? Drive.SLOW_TRANSLATION_METERS_PER_SECOND : Drive.FAST_ROTATION_RADIANS_PER_SECOND;
            // double rSpeed = slowmodeTeleop ? Drive.SLOW_ROTATION_RADIANS_PER_SECOND : Drive.FAST_ROTATION_RADIANS_PER_SECOND;

            // double xDesired = Util.powerCopySign(Util.handleDeadband(-RobotContainer.driver.getLeftY(), Drive.JOYSTICK_DRIVE_TOLERANCE), 2);
            // double yDesired = Util.powerCopySign(Util.handleDeadband(-RobotContainer.driver.getLeftX(), Drive.JOYSTICK_DRIVE_TOLERANCE), 2);
            // double rotDesired = Util.powerCopySign(Util.handleDeadband(-RobotContainer.driver.getRightX(), Drive.JOYSTICK_TURN_TOLERANCE), 2);

            boolean stop = translation.getNorm() == 0 && omega == 0;
            
            if (aimingTeleop)
                stop = stop && onTargetTeleop;

            tryingToDriveTeleop = !stop;
            boolean tryingToTurn = omega != 0;
            boolean shouldStabilize = stabilizationDebouncer.calculate(!tryingToTurn && !aimingTeleop && translation.getNorm() != 0);

            Logger.recordOutput("Swerve/ShouldStabilize", shouldStabilize);

            if (stop && !RobotContainer.isDriverRBPressed.getAsBoolean()) {
                RobotContainer.drive.acceptSwerveCommand(new SwerveCommand.SwerveDriveBrake());
            } else {
                acceptSwerveCommand(
                    new SwerveCommand.FieldCentric()
                        .withVelocityX(translation.getX())
                        .withVelocityY(translation.getY())
                        .withRotationalRate(omega)
                        .withCenterOfRotation(new Translation2d(Units.inchesToMeters(4.5), 0))
                );
                aimingTeleop = false;
            }

            lastJoystickCommand = translation;
            wasStabilizing = shouldStabilize;
        }, this).withName("driveTeleopNormal");
    }

    public Command driveSnappedToAngle(Supplier<Rotation2d> angle) {
        return Commands.run(() -> {
            Pair<Translation2d, Double> input = getTeleopInput();
            Translation2d translation = input.getFirst();

            RobotContainer.drive.acceptSwerveCommand(
                new SwerveCommand.FieldCentricFacingAngle()
                    .withVelocityX(translation.getX())
                    .withVelocityY(translation.getY())
                    .withTargetDirection(stabilizationHeading)
            );
        }, this);
    }

    public Command driveRobotCentric() {
        return Commands.run(() -> {
            Pair<Translation2d, Double> input = getTeleopInput();
            Translation2d translation = input.getFirst();
            double omega = input.getSecond().doubleValue();
                
            boolean reversed = RobotStatus.getInstance().getAllianceColor() == AllianceColor.RED;
            translation = translation.times(reversed ? -1 : 1);

            RobotContainer.drive.acceptSwerveCommand(
                new SwerveCommand.RobotCentric()
                    .withVelocityX(-translation.getX())
                    .withVelocityY(-translation.getY())
                    .withRotationalRate(omega)
            );
        }, this).withName("driveTeleopRobotCentric");
    }

    public Command driveTeleopLockedOnGoal() {
        return Commands.run(() -> {
            System.out.println("DRIVING TELEOP LOCKED");
            Pair<Translation2d, Double> input = getTeleopInput();
            Translation2d translation = input.getFirst();
            
            RobotContainer.drive.acceptSwerveCommand(
                new SwerveCommand.FieldCentricFacingAngle()
                    .withVelocityX(translation.getX())
                    .withVelocityY(translation.getY())
                    .withTargetDirection(ShotPlanner.getInstance().getAimTarget()));
        }, this).withName("driveTeleopLockedOnGoal");
    }

    // for shooting on the move
    public Command driveTeleopConstantVelocity() {
        return Commands.run(() -> {
            acceptSwerveCommand(
                new SwerveCommand.FieldCentricFacingAngle()
                    .withVelocityX(lastJoystickCommand.getX())
                    .withVelocityY(lastJoystickCommand.getY())
                    .withTargetDirection(ShotPlanner.getInstance().getAimTarget())
            );
        }, this).until(() -> getTeleopInput().getFirst().getNorm() == 0).withTimeout(3).withName("driveTeleopConstantVelocity");
    }

    public Command driveRadiallyFromGoal() {
        return driveTeleopConstantVelocity()
            .beforeStarting(() -> { 
                lastJoystickCommand = new Translation2d(lastJoystickCommand.getNorm() * Math.signum(lastJoystickCommand.getX()) * 0.5, ShotPlanner.getInstance().getAimTarget());
            });
    }

    public Command brake() {
        return Commands.runOnce(() -> acceptSwerveCommand(new SwerveCommand.SwerveDriveBrake()), this);
    }

    public Command snapToAngle(Supplier<Rotation2d> target) {
        return Commands.run(() -> {
            System.out.println("SNAPPING TO ANGLE");
            acceptSwerveCommand(
                new SwerveCommand.FieldCentricFacingAngle()
                    .withTargetDirection(target.get())
            );
        }, this).until(() -> Math.abs(getAngle().minus(target.get()).getDegrees()) < 2.5 || isTryingToDrive()).andThen(brake());
    }

    public Command snapToAngle(Rotation2d target) {
        return snapToAngle(() -> target);
        // return Commands.run(() -> {
        //     acceptSwerveCommand(
        //         new SwerveCommand.FieldCentricFacingAngle()
        //             .withTargetDirection(target)
        //     );
        // }, this).until(() -> Math.abs(getAngle().minus(target).getDegrees()) < 2.5 || isTryingToDrive()).andThen(brake());
    }

    public SwerveCommand getFetchingCommand() {
        Translation2d noteToRobot = RobotContainer.noteDetector.getBestNoteToRobot();
        Translation2d pickupPosition = new Translation2d(Units.inchesToMeters(8), 0); // relative to the center of the robot

        Translation2d delta = noteToRobot.minus(pickupPosition);
        double speed = -pickupController.calculate(delta.getNorm(), 0) * FAST_TRANSLATION_METERS_PER_SECOND/2;

        Translation2d movement = new Translation2d(speed, delta.getAngle());

        Logger.recordOutput("Seeking Angle", delta.getAngle().getDegrees());

        return (
            new SwerveCommand.RobotCentricFacingAngle()
                .withVelocityX(movement.getX())
                .withVelocityY(movement.getY())
                .withTargetDirection(delta.getAngle().times(1.75).plus(getAngle()))
        );
    }

    public Command chaseNote() {
        return Commands.run(() -> {
            if (!RobotContainer.noteDetector.noteInView()) {
                return;
            }
            acceptSwerveCommand(getFetchingCommand());
        }, this).until(Logic.not(RobotContainer.noteDetector::noteInView)).andThen(Commands.waitSeconds(0.1)).andThen(Commands.runOnce(() -> acceptSwerveCommand(new SwerveCommand.FieldCentric())));
    }

    public SwerveCommand getLineupCommand(Pose2d dest, double vFinal) {
        Translation2d delta = currentPose.getTranslation().minus(dest.getTranslation());

        double speed = (vFinal == 0.0)
                     ? dopController.calculate(delta.getNorm(), 0) * FAST_TRANSLATION_METERS_PER_SECOND/2
                     : -Math.abs(vFinal);

        Translation2d movement = new Translation2d(speed, delta.getAngle());

        return (
            new SwerveCommand.FieldCentricFacingAngle()
                .withVelocityX(movement.getX())
                .withVelocityY(movement.getY())
                .withTargetDirection(dest.getRotation())
        );
    }

    public Command driveToPoint(Pose2d dest, double vFinal) {
        return Commands.run(() -> {
            acceptSwerveCommand(getLineupCommand(dest, vFinal));
        }, this)
        .until(
            () -> currentPose.getTranslation().minus(dest.getTranslation()).getNorm() < Units.inchesToMeters(6)
        ); // command can't keep running, will affect other robot actions
    }

    public Command driveToPoint(Pose2d dest) {
        return driveToPoint(dest, 0.0);
    }

    public SwerveCommand followAprilTag() {
        Translation2d tag = RobotContainer.photon.bestTargetPose();
        Translation2d tagToRobot = currentPose.getTranslation().minus(tag);
        Translation2d pickupPosition = new Translation2d(Units.inchesToMeters(8), 0); // relative to the center of the robot

        Translation2d delta = tagToRobot.minus(pickupPosition);
        double speed = -followAprilTagController.calculate(delta.getNorm(), 0) * FAST_TRANSLATION_METERS_PER_SECOND/2;

        Translation2d movement = new Translation2d(speed, delta.getAngle());

        return (
            new SwerveCommand.RobotCentricFacingAngle()
                .withVelocityX(movement.getX())
                .withVelocityY(movement.getY())
                .withTargetDirection(delta.getAngle().times(1.75).plus(getAngle()))
        );
    }

    public Command chaseTag() {
        return Commands.run(() -> {
            if (RobotContainer.photon.maxTagsFromOneCam() == 0) {
                return;
            }
            acceptSwerveCommand(followAprilTag());
        }, this).until(() -> followAprilTagController.atSetpoint()).andThen(Commands.waitSeconds(0.1)).andThen(Commands.runOnce(() -> acceptSwerveCommand(new SwerveCommand.FieldCentric())));
    }

    public Command climbNudgeForward() {
        return Commands.run(() ->  {
            acceptSwerveCommand(
                new SwerveCommand.RobotCentric()
                    .withVelocityX(Units.inchesToMeters(15))
                    .withVelocityY(0)
                    .withRotationalRate(0)
            );
        }, this).withTimeout(1.75);
    }

    // public Command driveToPoint(Pose2d target) {
    //     return Commands.runOnce(
            
    //     );
    // }

    public Command snapToSpeaker() {
        return enableTeleopAim().andThen(snapToAngle(ShotPlanner.getInstance()::getAimTarget));
    }

    public Command aimTeleop() {
        return Commands.run(() -> {
            boolean moving = isTryingToDrive();
            SwerveCommand target;
            
            // if (moving) {
            //     System.out.println("DRIVING TELEOP LOCKED");
            Pair<Translation2d, Double> input = getTeleopInput();
            Translation2d translation = input.getFirst();
            
            target = new SwerveCommand.FieldCentricFacingAngle()
                .withVelocityX(translation.getX())
                .withVelocityY(translation.getY())
                .withTargetDirection(ShotPlanner.getInstance().getAimTarget());
            // } else {
            //     System.out.println("Snapping to angle");
            //     target = new SwerveCommand.FieldCentricFacingAngle()
            //         .withTargetDirection(ShotPlanner.getInstance().getAimTarget());
            // }

            acceptSwerveCommand(target);

        }, this);
    }

    public Command enableTeleopAim() {
        return Commands.runOnce(() -> aimingTeleop = true);
    }

    public Command disableTeleopAim() {
        return Commands.runOnce(() -> aimingTeleop = false);
    }

    public Command toggleSlowMode() {
        return Commands.runOnce(() -> slowmodeTeleop = !slowmodeTeleop);
    }

    public boolean isTryingToDrive() {
        return tryingToDriveTeleop;
    }
    
    public void acceptVisionMeasurement(EstimatedRobotPose visionPose) {
        Matrix<N3, N1> stdDevs = VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30));
        // if(visionPose.targetsUsed.size() == 1 && visionPose.targetsUsed.get(0).getPoseAmbiguity() > 0.1) {
        // if (visionPose.targetsUsed.size() == 1) {
        //     stdDevs = VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30));
            
        //     boolean tooFar = visionPose.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm() >= Units.feetToMeters(18);
        //     boolean tooAskew = Math.abs(visionPose.targetsUsed.get(0).getBestCameraToTarget().getY()) > Units.feetToMeters(2.5);
        //     boolean tooAmbiguous = visionPose.targetsUsed.get(0).getPoseAmbiguity() > 0.2;

        //     if(tooAmbiguous || tooFar || tooAskew) {
        //         return;
        //     }
        // }

        // if (!DriverStation.isTeleopEnabled()) {
        swerve.acceptVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds, stdDevs);
        // }
    }
    
    // For multiple camera pose calculation
    public void acceptVisionMeasurement(ArrayList<EstimatedRobotPose> visionPoses) {
        for (EstimatedRobotPose pose : visionPoses) {
            acceptVisionMeasurement(pose);
        }

        // ArrayList<EstimatedRobotPose> result = new ArrayList<>();
        // // All the visionPose should see the same number of tags
        // Matrix<N3, N1> stdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));
        // for (EstimatedRobotPose pose : visionPoses) {
        //     stdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));
            
        //     if (pose.targetsUsed.size() == 1) {
        //         stdDevs = VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(30));
                
        //         boolean tooFar = pose.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm() >= Units.feetToMeters(18);
        //         boolean tooAskew = Math.abs(pose.targetsUsed.get(0).getBestCameraToTarget().getY()) > Units.feetToMeters(2.5);
        //         boolean tooAmbiguous = pose.targetsUsed.get(0).getPoseAmbiguity() > 0.2;

        //         if(tooAmbiguous || tooFar || tooAskew) {
        //             continue;
        //         }
        //     }
        //     result.add(pose);
        // }
        
        
        // if (result.size() > 0) {
        //     Pose2d sum = new Pose2d();
        //     for (EstimatedRobotPose pose : result) {
        //         sum.plus(new Transform2d(pose.estimatedPose.toPose2d().getTranslation(), pose.estimatedPose.toPose2d().getRotation()));
        //     }
        //     sum.div(result.size());
        //     swerve.acceptVisionMeasurement(sum, result.get(0).timestampSeconds, stdDevs);
        // }
    }

    public Command getChoreoSwerveCommandFromTrajectory(String choreoFileName) {
        ChoreoTrajectory traj = Choreo.getTrajectory(choreoFileName);

        this.field.getObject("Trajectory").setPoses( traj.getPoses() );

        PIDController xController = new PIDController(1.5, 0.0, 0.0); // X controller (tune to keep staying on path)
        PIDController yController = new PIDController(1.5, 0.0, 0.0); // Y Controller (tune to keep staying on path)
        PIDController thetaController = new PIDController(3.0, 0.0, 0.0); // Heading controller (tune to keep staying on path)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Applies ChassisSpeeds on robot for path following (PID values are for correction during path)
        return Choreo.choreoSwerveCommand(
                traj,
                () -> this.currentPose,
                xController,
                yController,
                thetaController,
                (ChassisSpeeds speeds) -> this.acceptSwerveCommand(new SwerveCommand.ApplyChassisSpeeds().withSpeeds(speeds)),
                () -> false, // if you want to flip alliances or not (current alliance = RED)
                this
            );
    }

    public enum ModuleLocation {
        FrontLeft(0), FrontRight(1), BackLeft(2), BackRight(3), TestStandModule(-1);
        public int index;
        private ModuleLocation(int index) {
            this.index = index;
        }
    }
    
    public enum DrivetrainType {
        SWERVE,
        PHOENIX_SWERVE,
        SIM_SWERVE
    }
}