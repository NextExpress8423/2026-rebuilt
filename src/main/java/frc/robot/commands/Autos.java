// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.CANDriveSubsystem;

public final class Autos {
  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        driveSubsystem.driveArcade(() -> -0.5, () -> 0).withTimeout(.25),
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
        driveSubsystem.stop(),
        // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
        // total of 10 seconds
        ballSubsystem.spinUpCommand().withTimeout(1),
        ballSubsystem.launchCommand().withTimeout(9),
        // Stop running the launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop()));
  }

  private static final TrajectoryConfig getConstraints() {
    var autoVoltageConstaint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.DriveConstants.ksVolts,
            Constants.DriveConstants.kvVoltSecondsPerMeter,
            Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
        Constants.DriveConstants.kDriveKinematics,
        10);
    return new TrajectoryConfig(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
        Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.DriveConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstaint);
  }

  public static final Command TestTrobbio(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    TrajectoryConfig config = getConstraints();
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
        // new Translation2d(1, 1),
        // new Translation2d(1.5, -1)
        ),
        new Pose2d(3, 3, new Rotation2d(90)),
        config);
    return driveSubsystem.resetOdometryCommand(exampleTrajectory.getInitialPose())
        .andThen(driveSubsystem.followTrajectoryCommand(exampleTrajectory))
        .andThen(driveSubsystem.stop());
  }

  public static final Command blueHubAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    TrajectoryConfig config = getConstraints().setReversed(false);
    TrajectoryConfig configReverse = getConstraints().setReversed(true);

    Trajectory prepareToShootTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(3.57, 4.0, new Rotation2d(0)),
        List.of(),
        new Pose2d(3.0, 4.0, new Rotation2d(0)),
        configReverse);
    Trajectory prepareToClimbTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(-110.0)),
        List.of(),
        new Pose2d(1.5, 3.2, Rotation2d.fromDegrees(-180.0)),
        config);
    SmartDashboard.putString("Blue Hub Auto Starting Pose", prepareToShootTrajectory.getInitialPose().toString());
    return driveSubsystem.resetOdometryCommand(prepareToShootTrajectory.getInitialPose())
        .andThen(driveSubsystem.followTrajectoryCommand(prepareToShootTrajectory))
        //.andThen(ballSubsystem.spinUpCommand().withTimeout(1))
        //.andThen(ballSubsystem.launchCommand().withTimeout(4))
        //.andThen(ballSubsystem.runOnce(() -> ballSubsystem.stop()))
        .andThen(driveSubsystem.rotateToCommand(Rotation2d.fromDegrees(-110), false))
        .andThen(driveSubsystem.followTrajectoryCommand(prepareToClimbTrajectory))
        .andThen(driveSubsystem.stop());

        // .andThen(driveSubsystem.stopRepeatedly().raceWith(
        //   ballSubsystem.spinUpCommand().withTimeout(1)
        //     .andThen(ballSubsystem.launchCommand().withTimeout(4))
        //     .andThen(ballSubsystem.runOnce(() -> ballSubsystem.stop()))
        //   ))

  }

  public static final Command driveFowardFourMeters(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    TrajectoryConfig config = getConstraints().setReversed(false);
    Trajectory driveFourMetersTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(3, 2, Rotation2d.fromDegrees(90)),
        List.of(),
        new Pose2d(3, 6, Rotation2d.fromDegrees(90)),
        config);
    return driveSubsystem.resetOdometryCommand(driveFourMetersTrajectory.getInitialPose())
        .andThen(driveSubsystem.followTrajectoryCommand(driveFourMetersTrajectory))
        .andThen(driveSubsystem.stop());
  }

}
