package org.frogforce503.robot2024.hardware;

import org.frogforce503.robot2024.subsystems.drive.Drive.DrivetrainType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class RobotHardwareProgrammingBot extends RobotHardware {
    
    @Override
    public void initializeConstants() {
        this.backLeftName = "AM_BackLeft"; // names subject to change
        this.backRightName = "AM_BackRight";
        this.frontLeftName = "AM_FrontLeft";
        this.frontRightName = "AM_FrontRight";

        this.drivetrainType = DrivetrainType.SWERVE;
        this.moduleType = SWERVE_MODULE_TYPE.ANDY;
        this.swerveCANBus = "rio";

        this.gyroID = 9;
        this.gyroType = GYRO_TYPE.PIGEON_1;

        // TOOD: remeasure values
        this.kWheelbaseLength = Units.inchesToMeters(21.0);
        this.kWheelbaseWidth = Units.inchesToMeters(21.0); 

        // this.fullRobotLength = Units.inchesToMeters(33.5);
        // this.fullRobotWidth = Units.inchesToMeters(33.5);

        this.wheelDiameter = Units.inchesToMeters(4);
        
        // Swerve Module Positions (relative to the center of the drive base)
        kVehicleToFrontLeft = new Translation2d(kWheelbaseWidth / 2, kWheelbaseLength / 2);
        kVehicleToFrontRight = new Translation2d(kWheelbaseWidth / 2, -kWheelbaseWidth / 2);
        kVehicleToBackRight = new Translation2d(-kWheelbaseWidth / 2, -kWheelbaseLength / 2);
        kVehicleToBackLeft = new Translation2d(-kWheelbaseWidth / 2, kWheelbaseLength / 2);

        kModulePositions = new Translation2d[] { kVehicleToFrontLeft, kVehicleToFrontRight,
                kVehicleToBackLeft, kVehicleToBackRight };

        kinematics = new SwerveDriveKinematics(kModulePositions);
        
        this.autoXController = new PIDController(1.0, 0.0, 0.0);
        this.autoYController = new PIDController(1.0, 0.0, 0.0);
        this.autoThetaController = new PIDController(1.0, 0.0, 0.0);

        this.calculateDimensions();
    }    
}
