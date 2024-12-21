package org.frogforce503.robot2024.planners;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.math.PolynomialRegression;
import org.frogforce503.lib.util.SBUtil;
import org.frogforce503.robot2024.RobotContainer;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.subsystems.Wrist.Goal;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShotPlanner {

    private double[][] closeShotMap = new double[][] {
        {1.344846795511825, 23.0},
        {1.6690441180904934, 23.0},
        {2.094954463181612, 18.0},
        {2.412943228861661, 14.0},
        {2.8486990482173438, 11.0},
        {3.5032269667224476, 5.0},
        {3.8876293421167203, 3.0},
        {4.445787754245988, 0.0}
    };

    private double[][] farShotMap = new double[][] {
        {4.445787754245988, 0.0},
        {5.00723842532441, 1.0}
    };

    private double[][] closeShotMap_SHARP = new double[][] {
        {2.1324672626573, 14.0},
        {2.8537334495484594, 8.0},
        {3.254689827513935, 5.0},
        {3.9689686299949347, 1.5},
        {4.912072112494548, 0.0}
    };

    private double[][] farShotMap_SHARP = new double[][] {
        {4.912072112494548, 0.0},
        {5.284958865531819, 1.0},
        {6.5368835641926735, 2.5}
    };

    private PolynomialRegression wristReg, armReg, wristRegSharp, armRegSharp;

    // FRONT BATTER WRIST SHOTMAP (Distance, Wrist Angle)
    private double[][] shotMapValuesInverted = new double[][] {
        {1.344846795511825, 110},
        {1.6690441180904934, 115}
    };

    static InterpolatingDoubleTreeMap kDistanceToWristAngleInverted = new InterpolatingDoubleTreeMap();

    private boolean isInSharpLocation = false;
    private boolean isAtWingline = false;

    private Translation2d goalFixed;
    private Pose2d nextPose;

    // outputs
    private Rotation2d aimTarget;
    private double distanceTarget;

    private final double ANGLE_LOOKAHEAD_TIME = 0.25;
    private final double AIM_LOOKAHEAD_TIME = 0.5;

    @Deprecated public static final double BATTER_DISTANCE_RED = 1.331;
    @Deprecated public static final double PODIUM_DISTANCE_RED = 3.05; // 2.882846872657513;
    @Deprecated public static final double CENTRAL_SHOT_DISTANCE_RED = 4.0357;
    
    @Deprecated public static final double BATTER_DISTANCE_BLUE = 1.331;
    @Deprecated public static final double PODIUM_DISTANCE_BLUE = 3.002;
    @Deprecated public static final double CENTRAL_SHOT_DISTANCE_BLUE = 4.0357;

    private static ShotPlanner instance = null;

    public boolean isHintingWithAng = false;
    public Pair<Double, Double> currentHintAng = Pair.of(0.0, 0.0);

    public static enum Mode {
        NORMAL,
        SHOTMAP_TUNING,
        COBRA,
        FRONT_BATTER
    }

    public Mode currentMode = Mode.NORMAL;

    public static ShotPlanner getInstance() {
        if (instance == null) {
            instance = new ShotPlanner();
        }
        return instance;
    }

    public void populateMaps() {
        for (int k = 0; k < shotMapValuesInverted.length; k++) {
            kDistanceToWristAngleInverted.put(shotMapValuesInverted[k][0], shotMapValuesInverted[k][1]);
        }

        // Temporary arrays
        double[] first = new double[closeShotMap.length];
        double[] second = new double[closeShotMap.length];

        // Normal wrist shotmap
        for (int i = 0; i < closeShotMap.length; i++) {
            first[i] = closeShotMap[i][0];
            second[i] = closeShotMap[i][1];
        }

        wristReg = new PolynomialRegression(
            first, 
            second,
            5
        );

        // Normal arm shotmap
        for (int i = 0; i < farShotMap.length; i++) {
            first[i] = farShotMap[i][0];
            second[i] = farShotMap[i][1];
        }

        armReg = new PolynomialRegression(
            first, 
            second,
            5
        );

        // Sharp wrist shotmap
        for (int i = 0; i < closeShotMap_SHARP.length; i++) {
            first[i] = closeShotMap_SHARP[i][0];
            second[i] = closeShotMap_SHARP[i][1];
        }

        wristRegSharp = new PolynomialRegression(
            first, 
            second,
            5
        );

        // Sharp arm shotmap
        for (int i = 0; i < farShotMap_SHARP.length; i++) {
            first[i] = farShotMap_SHARP[i][0];
            second[i] = farShotMap_SHARP[i][1];
        }

        armRegSharp = new PolynomialRegression(
            first, 
            second,
            5
        );
    }

    public void initialize() {
        this.goalFixed = FieldConfig.getInstance().RED_SPEAKER;
        this.nextPose = new Pose2d();
        this.aimTarget = new Rotation2d();
        this.distanceTarget = 0.0;

        populateMaps();
        SBUtil.initShotmapHelper();
    }

    public void update() {
        this.goalFixed = RobotContainer.red()
                ? FieldConfig.getInstance().getTagById(4).getTranslation()
                : FieldConfig.getInstance().getTagById(7).getTranslation();

        Transform2d displacement = RobotContainer.drive.getVelocity().times(-ANGLE_LOOKAHEAD_TIME);
        nextPose = RobotContainer.drive.getPose().plus(displacement);

        Pose2d aimPose = (displacement.getTranslation().getNorm() < Units.inchesToMeters(7)
                && Math.abs(displacement.getRotation().getDegrees()) < 5)
                        ? RobotContainer.drive.getPose()
                        : nextPose;

        this.aimTarget = getRawAngleToGoal(aimPose.getTranslation()).plus(new Rotation2d(Math.PI));

        displacement = RobotContainer.drive.getVelocity().times(-AIM_LOOKAHEAD_TIME); // using a different lookahead time than angle
        nextPose = RobotContainer.drive.getPose().plus(displacement); // effectively a feedforward

        aimPose = (displacement.getTranslation().getNorm() < Units.inchesToMeters(7)
                && Math.abs(displacement.getRotation().getDegrees()) < 5)
                        ? RobotContainer.drive.getPose()
                        : nextPose;

        this.distanceTarget = getRawDistanceToGoal(aimPose.getTranslation());

        // Other variables
        this.isInSharpLocation = RobotContainer.drive.getPose().getY() <= FieldConfig.getInstance().NOTE_A.getY();
        this.isAtWingline = MathUtils.isInRange(
            FieldConfig.getInstance().BLUE_STAGE_LEFT.getX() - 0.5,
            FieldConfig.getInstance().RED_STAGE_RIGHT.getX() + 0.5,
            RobotContainer.drive.getPose().getX()
        );
        
        outputTelemetry();
    }

    public void setTuning(boolean set) {
        if (set) {
            setMode(Mode.SHOTMAP_TUNING);
        } else if (currentMode == Mode.SHOTMAP_TUNING) {
            setMode(Mode.NORMAL);
        }
    }

    public void setMode(Mode mode) {
        currentMode = mode;
    }

    public boolean isTuning() {
        return currentMode == Mode.SHOTMAP_TUNING;
    }

    private void outputTelemetry() {
        Logger.recordOutput("ShotPlanner/PredictedPose", nextPose);
        SmartDashboard.putNumber("Distance to BLUE Speaker", this.distanceTarget);
        SBUtil.updateShotmapHelper();
        
        Logger.recordOutput("ShotPlanner/AimTarget", new Pose3d(new Translation3d(this.goalFixed.getX(), this.goalFixed.getY(), Units.inchesToMeters(78)), new Rotation3d()));
        Logger.recordOutput("ShotPlanner/IsHinting", isHintingWithAng);
        Logger.recordOutput("ShotPlanner/AngleTarget", aimTarget.getDegrees());
        Logger.recordOutput("ShotPlanner/Mode", currentMode.name());
        Logger.recordOutput("ShotPlanner/Hint/Arm", currentHintAng.getFirst());
        Logger.recordOutput("ShotPlanner/Hint/Wrist", currentHintAng.getSecond());

        SmartDashboard.putNumber("DistToHailMaryTarget", 
            RobotContainer.drive.getPose().getTranslation().minus(
                goalFixed.plus(new Translation2d(RobotContainer.red() ? Units.feetToMeters(-6) : Units.feetToMeters(6), 2)
            ))
            .getNorm()
        );
    }

    public Rotation2d getAimTarget() {
        if (currentMode == Mode.FRONT_BATTER) {
            return this.aimTarget.plus(new Rotation2d(Math.PI));
        } else if (RobotContainer.wrist.currentGoal == Goal.AMP) {
            return new Rotation2d(-Math.PI/2);
        } else if (RobotContainer.wrist.currentGoal == Goal.HAIL_MARY) {
            Translation2d hailMaryGoal = RobotContainer.drive.getPose().getTranslation().minus(
                goalFixed.plus(new Translation2d(RobotContainer.red() ? Units.feetToMeters(-6) : Units.feetToMeters(6), 2)
            ));
            
            return hailMaryGoal.getAngle();
        } else if (this.isInSharpLocation) {
            return this.aimTarget.plus(Rotation2d.fromDegrees(8));
        } else if (this.isAtWingline) {
            return this.aimTarget.plus(Rotation2d.fromDegrees(3));
        }
        return this.aimTarget;
    }

    public double getDistanceTarget() {
        return this.distanceTarget;
    }

    public Rotation2d getRawAngleToGoal(Translation2d translation2d) {
        Translation2d delta = this.goalFixed.minus(translation2d);

        Logger.recordOutput("AimDisplacement", delta);
        return delta.getAngle();
    }

    public double getRawDistanceToGoal(Translation2d translation2d) {
        return Math.abs(this.goalFixed.minus(translation2d).getNorm());
    }

    public double getArmAngleForDistance(double distance) {
        boolean distInRange = MathUtils.isInRange(farShotMap[0][0], farShotMap[farShotMap.length - 1][0], distance);

        if (distInRange) {
            return MathUtil.clamp(
                (isInSharpLocation ? armRegSharp : armReg).predict(distance),
                RobotContainer.arm.MIN_POS,
                RobotContainer.arm.MAX_POS
            );
        }

        return 0.0;
    }

    public double getWristAngleForDistance(double distance) {
        boolean distInRange = MathUtils.isInRange(0.0, closeShotMap[closeShotMap.length - 1][0], distance);

        if (distInRange) {
            return MathUtil.clamp(
                (isInSharpLocation ? wristRegSharp : wristReg).predict(distance),
                RobotContainer.wrist.MIN_POS,
                RobotContainer.wrist.getWristMax()
            );
        }

        return 0.0;
    }

    private double armSetpoint(double distance) {
        switch (currentMode) {
            case NORMAL:
                return isHintingWithAng ? currentHintAng.getFirst() : this.getArmAngleForDistance(distance);
            case COBRA:
                return Presets.COBRA.armAngle;
            case SHOTMAP_TUNING:
                return SBUtil.getComponent("ShotPlanner", "Arm Angle ShotMap", 0);
            case FRONT_BATTER:
                return Presets.FRONT_BATTER.armAngle;
        }
        return 0;
    }

    public double getArmSetpoint() {
        return armSetpoint(this.distanceTarget);
    }

    private double wristSetpoint(double distance) {
        switch (currentMode) {
            case NORMAL:
                return (isHintingWithAng ? currentHintAng.getSecond() : this.getWristAngleForDistance(distance));
            case COBRA:
                return Presets.COBRA.wristAngle;
            case SHOTMAP_TUNING:
                return SBUtil.getComponent("ShotPlanner", "Wrist Angle ShotMap", 0);
            case FRONT_BATTER:
                return MathUtil.clamp(
                    kDistanceToWristAngleInverted.get(distance),
                    RobotContainer.wrist.MIN_POS,
                    RobotContainer.wrist.getWristMax()
                );
        }
        return 0;
    }

    public double getWristSetpoint() {
        return wristSetpoint(this.distanceTarget);
    }

    public void setHintWithAng(double armAng, double wristAng) {
        this.currentHintAng = Pair.of(armAng, wristAng);
        isHintingWithAng = true;
    }

    public void setHintWithAng(Presets preset) {
        this.setHintWithAng(preset.armAngle, preset.wristAngle);
    }

    public void setHintWithDistance(double distance) {
        this.setHintWithAng(getArmAngleForDistance(distance), getWristAngleForDistance(distance));
    }

    public void cancelHint() {
        this.isHintingWithAng = false;
    }

    public enum Presets {
        AMP(100,40),
        BATTER(0.0, 23.0),
        PODIUM(0.0, 6.0),
        CENTRAL(0.0, 0.0),
        HAIL_MARY(0.0, 30.0), //24.0
        FRONT_BATTER(8, 110),
        COBRA(90, 92.5);

        public final double armAngle, wristAngle;

        private Presets(double armAngle, double wristAngle) {
            this.armAngle = armAngle;
            this.wristAngle = wristAngle;
        }

        public double getArmAngle() {
            return armAngle;
        }

        public double getWristAngle() {
            return wristAngle;
        }
    }
}