// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {

  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;
  WPI_PigeonIMU gyro;
  private Field2d field = new Field2d();

  private final DifferentialDrive drive;

  // Mutable holders for unit-safe SysId logging - persisted to avoid reallocation
  // every loop
  private final MutVoltage appliedVoltage = Volts.mutable(0);
  private final MutDistance distance = Meters.mutable(0);
  private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);
  // Creates a SysIdRoutine
  SysIdRoutine routine;

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);
    kinematics = kDriveKinematics;
    gyro = new WPI_PigeonIMU(PIGEON_ID);
    
    SmartDashboard.putData("field", field);

    odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(-gyro.getAngle()),
        leftLeader.getEncoder().getPosition(),
        rightLeader.getEncoder().getPosition(),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(metersPerRotation);
    config.encoder.velocityConversionFactor(metersPerSecondConversion);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    config.inverted(true);

    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(false);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            voltage -> {
              leftLeader.setVoltage(voltage);
              rightLeader.setVoltage(voltage);
            },
            log -> {
              log.motor("driveLeft")
                  .voltage(
                      appliedVoltage.mut_replace(leftLeader.getAppliedOutput() * leftLeader.getBusVoltage(), Volts))
                  .linearPosition(distance.mut_replace(leftLeader.getEncoder().getPosition(), Meters))
                  .linearVelocity(velocity.mut_replace(leftLeader.getEncoder().getVelocity(), MetersPerSecond));
              log.motor("driveRight")
                  .voltage(
                      appliedVoltage.mut_replace(rightLeader.getAppliedOutput() * rightLeader.getBusVoltage(), Volts))
                  .linearPosition(distance.mut_replace(rightLeader.getEncoder().getPosition(), Meters))
                  .linearVelocity(velocity.mut_replace(rightLeader.getEncoder().getVelocity(), MetersPerSecond));
            }, this));
  }

  @Override
  public void periodic() {
    Pose2d pose = odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()),
        leftLeader.getEncoder().getPosition(),
        rightLeader.getEncoder().getPosition());
    field.setRobotPose(pose);

    var wheelSpeeds = getWheelSpeeds();
    SmartDashboard.putNumber("Left Wheel V", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right Wheel V", wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Left Wheel V Follow", leftFollower.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Wheel V Follow", rightFollower.getEncoder().getVelocity());
    SmartDashboard.putNumber("Left Wheel P", leftLeader.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Wheel P", rightLeader.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Power", leftLeader.getAppliedOutput());
    SmartDashboard.putNumber("Right Power", rightLeader.getAppliedOutput());
  }

  public void setPose(Pose2d newPose2d) {
    System.out.println("Setting pose: " + newPose2d.toString());
    SmartDashboard.putString("Last Pose Reset", newPose2d.toString());
    odometry.resetPose(newPose2d);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftLeader.getEncoder().getVelocity(),
        rightLeader.getEncoder().getVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    drive.feed();
  }

  public Command resetOdometryCommand(Pose2d newPose2d) {
    return this.runOnce(
        () -> setPose(newPose2d));
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  public Command stopRepeatedly() {
    return this.run(
      () -> drive.arcadeDrive(0.0, 0.0)
    );
  }

  public Command stop() {
    return this.runOnce(
        () -> drive.arcadeDrive(0, 0));
  }

  public Command rotateToCommand(Rotation2d heading, boolean isCCW) {

    return runEnd(
      () -> drive.arcadeDrive(0.0, isCCW ? 0.35 : -0.35), 
      () -> drive.arcadeDrive(0.0, 0.0)
    ).until(
      () -> Math.abs(getPose().getRotation().getDegrees() - heading.getDegrees()) < 5.0
    );
    // return new Command() {
    //     public void initialize() {
    //       drive.arcadeDrive(0.0, isCCW ? 0.1 : -0.1);
    //     }
    //     public void execute() {
    //       drive.feed();
    //     }
    //     public void end(boolean interrupted) {
    //       drive.arcadeDrive(0.0, 0.0);
    //     }
    //     public boolean isFinished() {
    //       return Math.abs(getPose().getRotation().getDegrees() - heading.getDegrees()) < 5.0;
    //     }
    //   };
  }

  public Command followTrajectoryCommand(Trajectory trajectory) {
    RamseteController controller = new RamseteController(
            Constants.DriveConstants.kRamseteB,
            Constants.DriveConstants.kRamseteZeta);
    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");
    controller.setEnabled(true);
    PIDController leftController = new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0);
    PIDController rightController = new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0);
    return runOnce(() -> field.getObject("traj").setTrajectory(trajectory))
      .andThen(new RamseteCommand(
        trajectory,
        this::getPose,
        controller,
        new SimpleMotorFeedforward(
            Constants.DriveConstants.ksVolts,
            Constants.DriveConstants.kvVoltSecondsPerMeter,
            Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
        kinematics,
        this::getWheelSpeeds,
        leftController,
        rightController,
        (leftVolts, rightVolts) -> {
          tankDriveVolts(leftVolts, rightVolts);
          leftMeasurement.setNumber(getWheelSpeeds().leftMetersPerSecond);
          rightMeasurement.setNumber(getWheelSpeeds().rightMetersPerSecond);
          leftReference.setNumber(leftController.getSetpoint());
          rightReference.setNumber(rightController.getSetpoint());
        },
        this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  // public Command Pigeon2() {
  // SmartDashboard.putString(null, null);
  // return this.run(null);
  // }
}
