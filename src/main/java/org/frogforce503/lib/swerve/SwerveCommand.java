package org.frogforce503.lib.swerve;

import org.frogforce503.robot2024.subsystems.drive.Drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Container for all the Swerve Requests. Use this to find all applicable swerve
 * drive requests.
 * <p>
 * This is also an interface common to all swerve drive control requests that
 * allow the
 * request to calculate the state to apply to the modules.
 */
public interface SwerveCommand extends SwerveRequest {

    public static class SwerveModuleCommand {
        public SwerveModuleState state;
        public boolean isOpenLoop;

        public SwerveModuleCommand(SwerveModuleState state, boolean isOpenLoop) {
            this.state = state;
            this.isOpenLoop = isOpenLoop;
        }
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply);
    public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters);
    public String requestInfo();

    /**
     * Sets the swerve drive module states to point inward on the
     * robot in an "X" fashion, creating a natural brake which will
     * oppose any motion.
     */
    public class SwerveDriveBrake implements SwerveCommand {

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagicExpo;

        public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters) {
            var states = new SwerveModuleStateEx[4];
            for (int i = 0; i < 4; ++i) {
                states[i] = new SwerveModuleStateEx(0, parameters.swervePositions[i].getAngle());
            }
            return states;
        }

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = getDriven(parameters);
            
            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public SwerveDriveBrake withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public SwerveDriveBrake withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

        public String requestInfo() {
            return "BrakeRequest";
        }
    }

    /**
     * Drives the swerve drivetrain in a field-centric manner.
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the field, and the rate at which their robot should rotate
     * about the center of the robot.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and RotationRate is 0.5 rad/s.
     * In this scenario, the robot would drive northward at 5 m/s and
     * turn counterclockwise at 0.5 rad/s.
     */
    public class FieldCentric implements SwerveCommand {
        /**
         * The velocity in the X direction.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The angular rate to rotate at.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         * <p>
         * This is in radians per second
         */
        public double RotationalRate = 0;
        /**
         * The allowable deadband of the request.
         */
        public double Deadband = 0;
        /**
         * Rotational deadband of the request
         */
        public double RotationalDeadband = 0;

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.Velocity;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        public Translation2d CenterOfRotation = new Translation2d();

        /**
         * The last applied state in case we don't have anything to drive
         */
        protected SwerveModuleState[] m_lastAppliedState = null;


        @Override
        public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            double toApplyOmega = RotationalRate;
            if(Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if(Math.abs(toApplyOmega) < RotationalDeadband) toApplyOmega = 0;

            ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                        Drive.getInstance().getAngle()), parameters.updatePeriod);
            
            var states = SwerveModuleStateEx.convertList(parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation));
            
            return states;
        }

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = getDriven(parameters);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        public FieldCentric withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        public FieldCentric withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        public FieldCentric withRotationalRate(double rotationalRate) {
            this.RotationalRate = rotationalRate;
            return this;
        }

        public FieldCentric withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }

        public FieldCentric withRotationalDeadband(double RotationalDeadband) {
            this.RotationalDeadband = RotationalDeadband;
            return this;
        }


        public FieldCentric withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public FieldCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public FieldCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

        public String requestInfo() {
            return "FieldCentric(" + VelocityX + ", " + VelocityY + ", " + RotationalRate + ")";
        }
    }

    /**
     * Drives the swerve drivetrain in a field-centric manner, maintaining a
     * specified heading
     * angle to ensure the robot is facing the desired direction
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the field, and the direction the robot should be facing.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and TargetDirection is 180
     * degrees.
     * In this scenario, the robot would drive northward at 5 m/s and
     * turn clockwise to a target of 180 degrees.
     * <p>
     * This control request is especially useful for autonomous control, where the
     * robot should be facing a changing direction throughout the motion.
     */
    public class FieldCentricFacingAngle implements SwerveCommand {
        /**
         * The velocity in the X direction.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The desired direction to face.
         * 0 Degrees is defined as in the direction of the X axis.
         * So a TargetDirection of 90 degrees will point along the Y axis, or to the
         * left.
         */
        public Rotation2d TargetDirection = new Rotation2d();

        /**
         * The allowable deadband of the request.
         */
        public double Deadband = 0;
        /**
         * Rotational deadband of the request
         */
        public double RotationalDeadband = 0;

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        /**
         * The PID controller used to maintain the desired heading.
         * Users can specify the PID gains to change how aggressively to maintain
         * heading.
         * <p>
         * This PID controller operates on heading radians and outputs a target
         * rotational rate in radians per second.
         */
        //Before tuning constants where kp:4.0, ki:0.0, kd:0.5
        //public PIDController HeadingController = new PIDController(3.8, 0.0, 0.5); // P: 10, 5
        
        public PhoenixPIDController HeadingController = new PhoenixPIDController(15, 0, .25); 
      
       
        public Translation2d CenterOfRotation = new Translation2d();

        @Override 
        public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            double rotationRate;

            // if (!HeadingController.atSetpoint()) {
            //     rotationRate = HeadingController.calculate(Drive.getInstance().getAngle().getRadians(),
            //         TargetDirection.getRadians(), parameters.timestamp);
            // } else {    
            //     rotationRate = 0.0;
            // }

            // System.out.println("Rotation rate: " + rotationRate);
 
            // double toApplyOmega = rotationRate;
           
            // if(Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
            //     toApplyX = 0;
            //     toApplyY = 0;
            // }

            // if(Math.abs(toApplyOmega) < RotationalDeadband) toApplyOmega = 0;

            double toApplyOmega = HeadingController.calculate(parameters.currentPose.getRotation().getRadians(),
            TargetDirection.getRadians(), parameters.timestamp);;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband || HeadingController.atSetpoint()) {
                toApplyOmega = 0;
            }

            // System.out.println(HeadingController.atSetpoint() + ", " + HeadingController.getPositionError() + ", " + HeadingController.getSetpoint() + ", " + toApplyOmega);

            ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                    Drive.getInstance().getAngle()), parameters.updatePeriod);

            var states = SwerveModuleStateEx.convertList(parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation));
            // SwerveModuleStateEx[] newStates = new SwerveModuleStateEx[states.length];

            return states;
        }

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = getDriven(parameters);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        public FieldCentricFacingAngle withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        public FieldCentricFacingAngle withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        public FieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
            this.TargetDirection = targetDirection;
            HeadingController.enableContinuousInput(-Math.PI, Math.PI);
            HeadingController.setTolerance(2.0 * Math.PI/180.0);
            return this;
        }

        public FieldCentricFacingAngle withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }
        public FieldCentricFacingAngle withRotationalDeadband(double RotationalDeadband) {
            this.RotationalDeadband = RotationalDeadband;
            return this;
        }

        public FieldCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public FieldCentricFacingAngle withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public FieldCentricFacingAngle withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

        public String requestInfo() {
            return "FieldCentricFacingAngle(" + VelocityX + ", " + VelocityY + ", " + TargetDirection.getDegrees() + ")";
        }
    }


    public class RobotCentricFacingAngle implements SwerveCommand {
        /**
         * The velocity in the X direction.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The desired direction to face.
         * 0 Degrees is defined as in the direction of the X axis.
         * So a TargetDirection of 90 degrees will point along the Y axis, or to the
         * left.
         */
        public Rotation2d TargetDirection = new Rotation2d();

        /**
         * The allowable deadband of the request.
         */
        public double Deadband = 0;
        /**
         * Rotational deadband of the request
         */
        public double RotationalDeadband = 0;

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        /**
         * The PID controller used to maintain the desired heading.
         * Users can specify the PID gains to change how aggressively to maintain
         * heading.
         * <p>
         * This PID controller operates on heading radians and outputs a target
         * rotational rate in radians per second.
         */
        public PhoenixPIDController HeadingController = new PhoenixPIDController(2, 0, 0);

        public Translation2d CenterOfRotation = new Translation2d();

        @Override
        public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;

            double rotationRate = HeadingController.calculate(Drive.getInstance().getAngle().getRadians(),
                    TargetDirection.getRadians(), parameters.timestamp);

            

            double toApplyOmega = rotationRate;
            if(Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if(Math.abs(toApplyOmega) < RotationalDeadband) toApplyOmega = 0;

            ChassisSpeeds speeds = ChassisSpeeds.discretize(new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega), parameters.updatePeriod);
            var states = SwerveModuleStateEx.convertList(parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation));

            return states;
        }

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = getDriven(parameters);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        public RobotCentricFacingAngle withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        public RobotCentricFacingAngle withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        public RobotCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
            this.TargetDirection = targetDirection;
            HeadingController.enableContinuousInput(-Math.PI, Math.PI);
            return this;
        }

        public RobotCentricFacingAngle withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }
        public RobotCentricFacingAngle withRotationalDeadband(double RotationalDeadband) {
            this.RotationalDeadband = RotationalDeadband;
            return this;
        }

        public RobotCentricFacingAngle withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public RobotCentricFacingAngle withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public RobotCentricFacingAngle withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

        public String requestInfo() {
            return "FieldCentricFacingAngle(" + VelocityX + ", " + VelocityY + ", " + TargetDirection.getDegrees() + ")";
        }
    }

    /**
     * Does nothing to the swerve module state. This is the default state of a newly
     * created swerve drive mechanism.
     */
    public class Idle implements SwerveCommand {

        @Override
        public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters) {
            return new SwerveModuleStateEx[] { new SwerveModuleStateEx(), new SwerveModuleStateEx(), new SwerveModuleStateEx(), new SwerveModuleStateEx()};
        }

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            return StatusCode.OK;
        }

        public String requestInfo() {
            return "Idle";
        }
    }

    /**
     * Sets the swerve drive modules to point to a specified direction.
     */
    public class PointWheelsAt implements SwerveCommand {

        /**
         * The direction to point the modules toward.
         * This direction is still optimized to what the module was previously at.
         */
        public Rotation2d ModuleDirection = new Rotation2d();

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;

        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        @Override
        public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters) {
            var states = new SwerveModuleStateEx[4];
            for (int i = 0; i < 4; ++i) {
                states[i] = new SwerveModuleStateEx(0, ModuleDirection);
            }
            return states;
        }

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = getDriven(parameters);
            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        public PointWheelsAt withModuleDirection(Rotation2d moduleDirection) {
            this.ModuleDirection = moduleDirection;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public PointWheelsAt withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public PointWheelsAt withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

        public String requestInfo() {
            return "PointWheelsAt(" + ModuleDirection.getDegrees() + ")";
        }
    }

    /**
     * Drives the swerve drivetrain in a robot-centric manner.
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the robot itself, and the rate at which their
     * robot should rotate about the center of the robot.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and RotationRate is 0.5 rad/s.
     * In this scenario, the robot would drive eastward at 5 m/s and
     * turn counterclockwise at 0.5 rad/s.
     */
    public class RobotCentric implements SwerveCommand {
        /**
         * The velocity in the X direction.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The angular rate to rotate at.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         * <p>
         * This is in radians per second
         */
        public double RotationalRate = 0;

        /**
         * The allowable deadband of the request.
         */
        public double Deadband = 0;
        /**
         * Rotational deadband of the request
         */
        public double RotationalDeadband = 0;

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;

        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        @Override
        public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            double toApplyOmega = RotationalRate;
            if(Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if(Math.abs(toApplyOmega) < RotationalDeadband) toApplyOmega = 0;
            ChassisSpeeds speeds = new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega);
            var states = SwerveModuleStateEx.convertList(parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d()));

            return states;
        }

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = getDriven(parameters);

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        public RobotCentric withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        public RobotCentric withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        public RobotCentric withRotationalRate(double rotationalRate) {
            this.RotationalRate = rotationalRate;
            return this;
        }

        public RobotCentric withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }
        public RobotCentric withRotationalDeadband(double RotationalDeadband) {
            this.RotationalDeadband = RotationalDeadband;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public RobotCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public RobotCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
        
        public String requestInfo() {
            return "RobotCentric(" + VelocityX + ", " + VelocityY + ", " + RotationalRate + ")";
        }
    }

    /**
     * Accepts a generic ChassisSpeeds to apply to the drivetrain.
     */
    public class ApplyChassisSpeeds implements SwerveCommand {

        /**
         * The chassis speeds to apply to the drivetrain.
         */
        public ChassisSpeeds Speeds = new ChassisSpeeds();
        /**
         * The center of rotation to rotate around.
         */
        public Translation2d CenterOfRotation = new Translation2d(0, 0);
        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        @Override
        public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters) {
            var states = parameters.kinematics.toSwerveModuleStates(this.Speeds, CenterOfRotation);
            return SwerveModuleStateEx.convertList(states);
        }

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        public ApplyChassisSpeeds withSpeeds(ChassisSpeeds speeds) {
            this.Speeds = speeds;
            return this;
        }

        public ApplyChassisSpeeds withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public ApplyChassisSpeeds withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public ApplyChassisSpeeds withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }

        public String requestInfo() {
            return "ChassisSpeeds(" + Speeds.toString() + ")";
        }
    }

    public class CharacterizationCommand implements SwerveCommand {

        private final MotionMagicVoltage stopCommand = new MotionMagicVoltage(0, false, 0, 0, false, false, false);
        private final VoltageOut outputCommand = new VoltageOut(0);
        private final Motor motor;

        private final boolean FOC;
        private double targetVoltage;

        public CharacterizationCommand(Motor motor, boolean FOC) {
            this.motor = motor;
            this.FOC = FOC;
        }

        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            for (SwerveModule module : modulesToApply) {
                if (this.motor == Motor.DRIVE) {
                    // double speed = (targetVoltage / 12.0) * Robot.bot.frontLeftConstants.SpeedAt12VoltsMps;
                    // module.apply(new SwerveModuleState(Units.MetersPerSecond.of(speed), new Rotation2d()), DriveRequestType.OpenLoopVoltage);
                    // SwerveModuleState openLoopState = new SwerveModuleState(speed , )
                    module.getSteerMotor().setControl(stopCommand);
                    module.getDriveMotor().setControl(outputCommand.withEnableFOC(FOC).withOutput(targetVoltage));
                } else {
                    module.getSteerMotor().setControl(outputCommand.withEnableFOC(FOC).withOutput(targetVoltage));
                    module.getDriveMotor().setControl(new VoltageOut(0));
                }
            }
            return StatusCode.OK;
        }

        public CharacterizationCommand withTargetVoltage(double voltage) {
            this.targetVoltage = voltage;
            return this;
        }

        @Override
        public SwerveModuleStateEx[] getDriven(SwerveControlRequestParameters parameters) {
           return SwerveModuleStateEx.convertList(parameters.kinematics.toSwerveModuleStates(new ChassisSpeeds()));
        }
        
        @Override
        public String requestInfo() {
            return "Characterization" + motor.name();
        }

        public static enum Motor {
            DRIVE,
            STEER
        }
    }
}