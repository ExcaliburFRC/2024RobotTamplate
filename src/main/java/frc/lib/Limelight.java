package frc.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.function.BiConsumer;

public class Limelight {
    PhotonCamera ll = new PhotonCamera("Limelight");
    AprilTagFieldLayout fieldLayout;

    public static Limelight INSTANCE = new Limelight();

    private Limelight() {
        ll.setDriverMode(false);

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
            throw new RuntimeException(e);
        }
    }


    public void setDriverMode(boolean isDriverMode){
        ll.setDriverMode(isDriverMode);
    }

    public void setPipeline(int index){
        ll.setPipelineIndex(index);
    }

    public PhotonPipelineResult getLatestResualt(){
        var result = ll.getLatestResult();

        if (result != null) return result;
        return new PhotonPipelineResult();
    }

    public boolean updateFromAprilTagPose(BiConsumer<Pose2d, Double> toUpdate) {
        var result = ll.getLatestResult();
        if (!result.hasTargets()) return false;

        var id = result.getBestTarget().getFiducialId();
        if (id == -1) return false;

        var tag = fieldLayout.getTagPose(id);
        if (tag.isEmpty()) return false;

        toUpdate.accept(tag.get().plus(result.getBestTarget().getBestCameraToTarget()).toPose2d(), result.getTimestampSeconds());
        return true;
    }
}

