package org.frogforce503.robot2024.subsystems.drive;

import org.frogforce503.robot2024.Robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SwerveVisualizer {
    private final double MaxSpeed;
    private final double visualSize = 3;
    private final double fillPercent = 0.75;
    private MechanismLigament2d[] m_moduleSpeeds;
    private MechanismLigament2d robotDirection;

    public SwerveVisualizer(double maxSpeed) {
        MaxSpeed = maxSpeed;
    }

    Color8Bit stationaryColor = new Color8Bit(Color.kWhite);
    Color8Bit speedColor = new Color8Bit(Color.kLightGreen);
    Color8Bit maxColor = new Color8Bit(Color.kGreen);

    public void initialize() {
        /* Mechanisms to represent the swerve module states */
        Mechanism2d visual = new Mechanism2d(visualSize, visualSize, new Color8Bit(Color.kBlack));

        double smallestDimension = Math.min(Robot.bot.kWheelbaseLength, Robot.bot.kWheelbaseWidth);
        double width = (Robot.bot.kWheelbaseWidth / smallestDimension) * fillPercent * visualSize;
        double length = (Robot.bot.kWheelbaseLength / smallestDimension) * fillPercent * visualSize;

        Translation2d bottomLeft = new Translation2d((visualSize - width)/2, (visualSize - length)/2);
        Translation2d topRight = new Translation2d((visualSize + width)/2, (visualSize + length)/2);

        // FL, FR, BL, BR
        // FR, BR, FL, BL

        m_moduleSpeeds = new MechanismLigament2d[] {
            visual.getRoot("FL_Speed", bottomLeft.getX(), topRight.getY()).append(new MechanismLigament2d("Speed", 0.5, 0, 10, speedColor)),
            visual.getRoot("FR_Speed", topRight.getX(), topRight.getY()).append(new MechanismLigament2d("Speed", 0.5, 0, 10, speedColor)),
            visual.getRoot("BL_Speed", bottomLeft.getX(), bottomLeft.getY()).append(new MechanismLigament2d("Speed", 0.5, 0, 10, speedColor)),
            visual.getRoot("BR_Speed", topRight.getX(), bottomLeft.getY()).append(new MechanismLigament2d("Speed", 0.5, 0, 10, speedColor)),
        };

        robotDirection = visual.getRoot("compass", visualSize/2, visualSize/2).append(new MechanismLigament2d("Angle", 0.5, 0, 25, new Color8Bit(Color.kYellow)));
        
        // m_moduleDirections = new MechanismLigament2d[] {
        //     visual.getRoot("FL_Direction", bottomLeft.getX(), topRight.getY())
        //             .append(new MechanismLigament2d("Direction", 0.1, 0, 0, directionColor)),
        //     visual.getRoot("FR_Direction", topRight.getX(), topRight.getY())
        //             .append(new MechanismLigament2d("Direction", 0.1, 0, 0, directionColor)),
        //     visual.getRoot("BL_Direction", bottomLeft.getX(), bottomLeft.getY())
        //             .append(new MechanismLigament2d("Direction", 0.1, 0, 0, directionColor)),
        //     visual.getRoot("BR_Direction", topRight.getX(), bottomLeft.getY())
        //             .append(new MechanismLigament2d("Direction", 0.1, 0, 0, directionColor)),
        // };

        SmartDashboard.putData("Swerve/Visualizer", visual);
    }

    Color8Bit[] colors = {new Color8Bit(Color.kGreen), new Color8Bit(Color.kBlue), new Color8Bit(Color.kRed), new Color8Bit(Color.kYellow)};
    // FL, FR, BL, BR

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void update(SwerveModuleState[] moduleStates, Rotation2d angle) {
        /* Telemeterize the module's states */
        for (int i = 0; i < 4; ++i) {
            // boolean moving = moduleStates[i].speedMetersPerSecond > Units.inchesToMeters(3);
            m_moduleSpeeds[i].setColor(colors[i]);
            // m_moduleSpeeds[i].setColor(moving ? (moduleStates[i].speedMetersPerSecond >= MaxSpeed - 0.2 ? maxColor : speedColor) : stationaryColor);
            m_moduleSpeeds[i].setAngle(moduleStates[i].angle.plus(Rotation2d.fromDegrees(90)));
            m_moduleSpeeds[i].setLength(moduleStates[i].speedMetersPerSecond / (2 * MaxSpeed) + 0.05);
        }
        
        robotDirection.setAngle(angle.plus(Rotation2d.fromDegrees(90)));
    }
}
