package org.frogforce503.robot2024.subsystems;

import java.util.AbstractMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.robot2024.Robot;
import org.frogforce503.robot2024.RobotContainer;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {
    private PhotonCamera frontLeftCamera, frontRightCamera, backLeftCamera, backRightCamera, backCamera;

    private PhotonPipelineResult result;
    private PhotonTrackedTarget bestTarget;

    NetworkTableInstance inst;
    NetworkTable photonVisionTable, frontTable, backTable;
    DoubleSubscriber distanceSub;
    DoublePublisher distancePub;

    private HashMap<CameraPos, EstimatedRobotPose> cameraPoses;

    public enum CameraPos {
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, BACK;
    }

    int frontLeftTags, frontRightTags, backLeftTags, backTags, backRightTags;

    private final PhotonPoseEstimator frontLeftPhotonEstimator;
    private final PhotonPoseEstimator backLeftPhotonEstimator;
    private final PhotonPoseEstimator frontRightPhotonEstimator;
    private final PhotonPoseEstimator backPhotonEstimator;
    private final PhotonPoseEstimator backRightPhotonEstimator;

    private EstimatedRobotPose frontLeftCameraPose, backLeftCameraPose, frontRightCameraPose, backCameraPose, backRightCameraPose;

    private static Photon instance = null;

    public static Photon getInstance() {
        if (instance == null) { instance = new Photon(); }
        return instance;
    }

    public Photon() {
        frontLeftCamera = new PhotonCamera("FrontLeftCamera");
        backLeftCamera = new PhotonCamera("BackLeftCamera");
        frontRightCamera = new PhotonCamera("FrontRightCamera");
        backCamera = new PhotonCamera("BackCamera");
        backRightCamera = new PhotonCamera("BackRightCamera");

        frontLeftPhotonEstimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontLeftCamera, Robot.bot.FRONT_LEFT_CAMERA_TO_CENTER);
        frontLeftPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        backLeftPhotonEstimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeftCamera, Robot.bot.BACK_LEFT_CAMERA_TO_CENTER);
        backLeftPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        frontRightPhotonEstimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontRightCamera, Robot.bot.FRONT_RIGHT_CAMERA_TO_CENTER);
        frontRightPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        backPhotonEstimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, Robot.bot.BACK_CAMERA_TO_CENTER);
        backPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        backRightPhotonEstimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backRightCamera, Robot.bot.BACK_RIGHT_CAMERA_TO_CENTER);
        backRightPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        inst = NetworkTableInstance.getDefault();
        photonVisionTable = inst.getTable("photonvision");
        backTable = photonVisionTable.getSubTable("back_camera");
        frontTable = photonVisionTable.getSubTable("front_camera");
        distancePub = photonVisionTable.getDoubleTopic("distance").publish();
        distanceSub = photonVisionTable.getDoubleTopic("distance").subscribe(0);

        cameraPoses = new HashMap<CameraPos, EstimatedRobotPose>();
    }

    public void displayCameraPositions() {
        Pose2d curPose = RobotContainer.drive.getPose();
        Pose3d robot = new Pose3d(curPose);

        Logger.recordOutput("Camera_FrontLeft", robot.plus(frontLeftPhotonEstimator.getRobotToCameraTransform()));
        Logger.recordOutput("Camera_BackLeft", robot.plus(backLeftPhotonEstimator.getRobotToCameraTransform()));
        Logger.recordOutput("Camera_FrontRight", robot.plus(frontRightPhotonEstimator.getRobotToCameraTransform()));
        Logger.recordOutput("Camera_BackCenter", robot.plus(backPhotonEstimator.getRobotToCameraTransform()));
        Logger.recordOutput("Camera_BackRight", robot.plus(backRightPhotonEstimator.getRobotToCameraTransform()));
    }

    public void updateTargets() {

        var frontLeftResult = frontLeftPhotonEstimator.update();
        if (frontLeftResult.isPresent()) {
            frontLeftCameraPose = frontLeftResult.get();
        } else if (frontLeftCameraPose != null) {
            frontLeftCameraPose = Timer.getFPGATimestamp() - frontLeftCameraPose.timestampSeconds < 0.25 ? frontLeftCameraPose : null;
            // Logger.recordOutput("Vision/FrontCameraDelta", Timer.getFPGATimestamp() - frontLeftCameraPose.timestampSeconds);
        }
        
        var backLeftResult = backLeftPhotonEstimator.update();
        if (backLeftResult.isPresent()) {
            backLeftCameraPose = backLeftResult.get();
        } else if (backLeftCameraPose != null) {
            backLeftCameraPose = Timer.getFPGATimestamp() - backLeftCameraPose.timestampSeconds < 0.25 ? backLeftCameraPose : null;
            // Logger.recordOutput("Vision/BackCameraDelta", Timer.getFPGATimestamp() - backLeftCameraPose.timestampSeconds);
        }

         var frontRightResult = frontRightPhotonEstimator.update();
        if (frontRightResult.isPresent()) {
            frontRightCameraPose = frontRightResult.get();
        } else if (frontRightCameraPose != null) {
            frontRightCameraPose = Timer.getFPGATimestamp() - frontRightCameraPose.timestampSeconds < 0.25 ? frontRightCameraPose : null;
            // Logger.recordOutput("Vision/FrontCameraDelta", Timer.getFPGATimestamp() - frontLeftCameraPose.timestampSeconds);
        }
        
        var backResult = backPhotonEstimator.update();
        if (backResult.isPresent()) {
            backCameraPose = backResult.get();
        } else if (backCameraPose != null) {
            backCameraPose = Timer.getFPGATimestamp() - backCameraPose.timestampSeconds < 0.25 ? backCameraPose : null;
        }
        
        var backRightResult = backRightPhotonEstimator.update();
        if (backRightResult.isPresent()) {
            backRightCameraPose = backRightResult.get();
        } else if (backRightCameraPose != null) {
            backRightCameraPose = Timer.getFPGATimestamp() - backRightCameraPose.timestampSeconds < 0.25 ? backRightCameraPose : null;
        }
    }

    @Override
    public void periodic() {
        updateTargets();
        displayCameraPositions();
    
        var frontLeft = frontLeftCameraPose;
        var backLeft = backLeftCameraPose;
        var frontRight = frontRightCameraPose;
        var back = backCameraPose;
        var backRight = backRightCameraPose;

        if (frontLeft != null) {
            RobotContainer.drive.getField().getObject("VisionFrontLeft").setPose(frontLeft.estimatedPose.toPose2d());
        }

        if (backLeft != null) {
            RobotContainer.drive.getField().getObject("VisionBackLeft").setPose(backLeft.estimatedPose.toPose2d());
        }

        if (frontRight != null) {
            RobotContainer.drive.getField().getObject("VisionFrontRight").setPose(frontRight.estimatedPose.toPose2d());
        }

        if (back != null) {
            RobotContainer.drive.getField().getObject("VisionBack").setPose(back.estimatedPose.toPose2d());
        }

        if (backRight != null) {
            RobotContainer.drive.getField().getObject("VisionBackRight").setPose(backRight.estimatedPose.toPose2d());
        }

        frontLeftTags = frontLeft == null ? 0 : frontLeft.targetsUsed.size();
        backLeftTags = backLeft == null ? 0 : backLeft.targetsUsed.size();
        frontRightTags = frontRight == null ? 0 : frontRight.targetsUsed.size();
        backTags = back == null ? 0 : back.targetsUsed.size();
        backRightTags = backRight == null ? 0 : backRight.targetsUsed.size();

        cameraPoses.clear();
        cameraPoses.put(CameraPos.FRONT_LEFT, frontLeftCameraPose);
        cameraPoses.put(CameraPos.FRONT_RIGHT, frontRightCameraPose);
        cameraPoses.put(CameraPos.BACK_LEFT, backLeftCameraPose);
        cameraPoses.put(CameraPos.BACK, backCameraPose);
        cameraPoses.put(CameraPos.BACK_RIGHT, backRightCameraPose);
        
        double minDist = getAverageCameraToTargetDistance(cameraPoses.get(CameraPos.BACK));
        Map.Entry<CameraPos, EstimatedRobotPose> mostReliablePose = new AbstractMap.SimpleEntry<CameraPos, EstimatedRobotPose>(CameraPos.BACK, cameraPoses.get(CameraPos.BACK));

        for (Map.Entry<CameraPos, EstimatedRobotPose> entry : cameraPoses.entrySet()) {
            double cameraDist = getAverageCameraToTargetDistance(entry.getValue());
            if (cameraDist < minDist) {
                mostReliablePose = entry;
                minDist = cameraDist;
            }
        }

        CameraPos bestEstCamera = mostReliablePose.getKey();
        EstimatedRobotPose bestEstPose = mostReliablePose.getValue();

        if (RobotBase.isReal() && bestEstPose != null) {
            PhotonTrackedTarget firstTar = bestEstPose.targetsUsed.get(0);

            boolean tooFar = firstTar.getBestCameraToTarget().getTranslation().getNorm() >= Units.feetToMeters(18);
            boolean tooAskew = Math.abs(firstTar.getBestCameraToTarget().getY()) > Units.feetToMeters(2.5);
            boolean tooAmbiguous = firstTar.getPoseAmbiguity() > 0.2;

            boolean tarUsed_1 = bestEstPose.targetsUsed.size() == 1 && !tooAmbiguous && !tooFar && !tooAskew;
            boolean tarUsed_2_OR_GREATER = bestEstPose.targetsUsed.size() >= 2;

            if (tarUsed_1 || tarUsed_2_OR_GREATER) { // not really necessary as bestEstPose != null means at least 1 tag being detected
                RobotContainer.drive.acceptVisionMeasurement(bestEstPose);
                Logger.recordOutput("Vision/UsingCamera", bestEstCamera);
                Logger.recordOutput("Vision/TagID", firstTar.getFiducialId());
            }
        }

        Logger.recordOutput("Vision/FrontLeftTags", frontLeftTags);
        Logger.recordOutput("Vision/FrontRightTags", frontRightTags);
        Logger.recordOutput("Vision/BackLeftTags", backLeftTags);
        Logger.recordOutput("Vision/BackRightTags", backRightTags);
        Logger.recordOutput("Vision/BackCenterTags", backTags);
    }

    public int maxTagsFromOneCam() {
        return (int) MathUtils.maxOf(frontLeftTags, frontRightTags, backLeftTags, backTags, backRightTags);
    }

    public boolean isTargetVisible() {
        if (this.result == null) {
            return false;
        }
        
        boolean res = false;

        try {
            res = this.result.hasTargets();
        } catch (Exception e) {
            System.out.println("Photonvision error: Reported from method isTargetVisible()");
            System.out.println("The error is: " + e);
            e.printStackTrace();
        }

        return res;
    }

    public boolean frontLeftCameraStatus() {
        return frontLeftCamera.isConnected();
    }

    public boolean backLeftCameraStatus() {
        return backLeftCamera.isConnected();
    }

    public boolean frontRightCameraStatus() {
        return frontRightCamera.isConnected();
    }

    public boolean backRightCameraStatus() {
        return backRightCamera.isConnected();
    }

    public boolean backCameraStatus() {
        return backCamera.isConnected();
    }

    public double distance() {
        return isTargetVisible()
             ? bestTarget.getBestCameraToTarget().getTranslation().getDistance(new Translation3d())
             : 0.0;
    }

    public Translation2d bestTargetPose() {
        return isTargetVisible()
             ? bestTarget.getBestCameraToTarget().getTranslation().toTranslation2d()
             : new Translation2d();
    }

    public void publishDistance() {
        distancePub.set(this.distance());
    }

    private double getAverageCameraToTargetDistance(EstimatedRobotPose cameraPose) {
        if (cameraPose == null) {
            return Double.MAX_VALUE;
        }

        List<PhotonTrackedTarget> targets = cameraPose.targetsUsed;
        
        if (targets.size() == 0) {
            return Double.MAX_VALUE;
        }

        return targets.stream().mapToDouble(
                target -> target.getBestCameraToTarget().getTranslation().getNorm()
            ).average().getAsDouble();
    }
}