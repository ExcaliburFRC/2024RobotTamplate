package frc.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.Optional;
import java.util.function.BiConsumer;

public class Limelight {
    PhotonCamera camera = new PhotonCamera("limelight");
    AprilTagFieldLayout fieldLayout;

    private PhotonPoseEstimator photonPoseEstimator;

    private final Transform3d robotToCamera = new Transform3d();

    public static Limelight INSTANCE;// = new Limelight();

    private Limelight() {
        camera.setDriverMode(false);

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCamera);
        } catch (IOException e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            throw new RuntimeException(e);
        }
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public void setDriverMode(boolean isDriverMode){
        camera.setDriverMode(isDriverMode);
    }

    public void setPipeline(int index){
        camera.setPipelineIndex(index);
    }

    public PhotonPipelineResult getLatestResualt(){
        var result = camera.getLatestResult();

        if (result != null) return result;
        return new PhotonPipelineResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}

