
package frc.robot.util;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubTargeting {
    private static final Translation2d BLUE_HUB_POSE = new Translation2d(4.66, 4.0);
    private static final Translation2d RED_HUB_POSE = new Translation2d(11.95, 4.0);

    private static final double BLUE_FEED_X = 1.0;
    private static final double RED_FEED_X = 15.0;
    private static final double FEEDING_TOLERANCE_METERS = 0.2;

    private Supplier<Pose2d> poseSupplier;
    private Translation2d hubPose = null;
    private double feedX = 1.0;
    private Function<Pose2d, Boolean> isFeedPosition = (pose) -> false;

    public HubTargeting(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public Translation2d getHubPose() {
        if (hubPose == null || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                if (alliance == Alliance.Blue) {
                    hubPose = BLUE_HUB_POSE;
                    feedX = BLUE_FEED_X;
                    isFeedPosition = (pose) -> pose.getX() > BLUE_HUB_POSE.getX() + FEEDING_TOLERANCE_METERS;
                } else {
                    hubPose = RED_HUB_POSE;
                    feedX = RED_FEED_X;
                    isFeedPosition = (pose) -> pose.getX() < RED_HUB_POSE.getX() - FEEDING_TOLERANCE_METERS;
                }
            });
        }
        return hubPose != null ? hubPose : BLUE_HUB_POSE;
    }

    public Rotation2d getAngleToHub() {
        Pose2d robotPose = poseSupplier.get();
        Translation2d diff = getHubPose().minus(robotPose.getTranslation());
        SmartDashboard.putString("HubPose", hubPose.toString());
        SmartDashboard.putString("TargetingBotPose", robotPose.toString());
        SmartDashboard.putString("TargetingDiff", diff.toString());
        SmartDashboard.putNumber("TargetingAngle", diff.getAngle().getDegrees());

        return diff.getAngle();
    }

    public double getDistanceToHub() {
        Pose2d robotPose = poseSupplier.get();
        if (isFeedPosition.apply(robotPose)) {
            return getFeedDistance(robotPose);
        }
        return getHubPose().minus(poseSupplier.get().getTranslation()).getNorm();
    }

    private double getFeedDistance(Pose2d robotPose) {
        return Math.abs(robotPose.getX() - feedX);
    }

}
