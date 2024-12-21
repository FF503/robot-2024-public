package org.frogforce503.robot2024.subsystems.drive.modules;

import org.frogforce503.robot2024.hardware.RobotHardware.SWERVE_MODULE_TYPE;
import org.frogforce503.robot2024.subsystems.drive.SwerveModuleLoader;
import org.frogforce503.robot2024.subsystems.drive.SwerveModuleLoader.SwerveModuleConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BaseSwerveModule extends SubsystemBase {
    public String moduleName;
    public ModuleLocation location;
    public String locationName;

    protected String debugName;
    protected String telemetryPrefix;

    public BaseSwerveModule(String moduleName, ModuleLocation location, SWERVE_MODULE_TYPE type) {
        this.moduleName = moduleName;
        this.location = location;
        this.debugName = location.name() + "(" + moduleName + ", " + type.name() + ")";
        this.locationName = location.name();
        this.telemetryPrefix = "Swerve/Modules/" + this.locationName + "/";

        SwerveModuleLoader.getInstance().setupModule(this);
    }

    public abstract void configure(SwerveModuleConfig config);

    public abstract void initialize();
    public abstract void update();

    // core module functionality

    public abstract void setRotationSetpoint(Rotation2d angle);
    public abstract void setDriveSetpoint(double speedMetersPerSecond);
    public abstract void setBrakeMode(boolean enable);

    public void setSwerveModuleState(SwerveModuleState state) {
        setRotationSetpoint(state.angle);
        setDriveSetpoint(state.speedMetersPerSecond);
    }

    public abstract Rotation2d getAngle();
    public abstract double getVelocity();
    public abstract double getDistance();
    
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }
    
    @Override
    public void periodic() {
        String prefix = "Swerve/Modules/" + this.locationName + "/";

        SmartDashboard.putNumber(prefix + "Azimuth/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(prefix + "Drive/Speed", getVelocity());
        SmartDashboard.putString(prefix + "Name", debugName);
    }

    public static enum ModuleLocation {
        FrontLeft(0), FrontRight(1), BackLeft(2), BackRight(3), TestStandModule(-1);
        public int index;
        private ModuleLocation(int index) {
            this.index = index;
        }
    }
}
