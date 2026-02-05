
package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubTargeting {
    private static final Translation2d BLUE_HUB_POSE = new Translation2d(4.0, 4.0);
    private static final Translation2d RED_HUB_POSE = new Translation2d(12.0, 4.0);

    private Supplier<Pose2d> poseSupplier;
    private Translation2d hubPose;

    public HubTargeting(Alliance alliance, Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        if (alliance == Alliance.Blue) {
            hubPose = BLUE_HUB_POSE;
        } else {
            hubPose = RED_HUB_POSE;
        }
    }

    public Rotation2d getAngleToHub() {
        return hubPose.minus(poseSupplier.get().getTranslation()).getAngle();
    }

    public double getDistanceToHub() {
        return hubPose.minus(poseSupplier.get().getTranslation()).getNorm();
    }

}
