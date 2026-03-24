// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

import java.util.function.Supplier;

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final TalonFX launcher;
  private final SparkMax intake;

  // change to whatever it should be, and move its own subsystem :/
  private final SparkMax climberMotor = new SparkMax(21, MotorType.kBrushless);

  private final VelocityVoltage launcherVelocityRequest;

  private Supplier<Double> distanceToHubSupplier;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem(Supplier<Double> distanceToHubSupplier) {
    // create brushed motors for each of the motors on the launcher mechanism
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    launcher = new TalonFX(LAUNCHER_MOTOR_ID);
    launcherVelocityRequest = new VelocityVoltage(0.0).withSlot(0);
    intake = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);

    this.distanceToHubSupplier = distanceToHubSupplier;

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching intake roller value", LAUNCHING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("launcher RPM", 0); // 0 is auto - Set non-zero to use fixed RPM
    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller

    TalonFXConfiguration LauncherConfig = new TalonFXConfiguration();

    LauncherConfig.CurrentLimits.SupplyCurrentLimit = (FEEDER_MOTOR_CURRENT_LIMIT);
    LauncherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    LauncherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    LauncherConfig.Slot0.kS = 0.14;
    LauncherConfig.Slot0.kV = 0.1075;
    LauncherConfig.Slot0.kA = 0.0;
    LauncherConfig.Slot0.kP = 0.2;
    LauncherConfig.Slot0.kI = 0.0;
    LauncherConfig.Slot0.kD = 0.0;
    launcher.getConfigurator().apply(LauncherConfig);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(true);
    feederConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.inverted(false);
    intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private double calculateRPM(double distanceToHub) {
    // y=488.69779x+3166.64619
    // y=-16.61362x^{3}+97.94771x^{2}+408.01527x+3088.93403
    // y=69.68989x^{3}-190.18269x^{2}+327.0665x+3585.34004
    // return 477.69779 * distanceToHub + 3174.64619;
    //69.68989 * Math.pow(distanceToHub, 3) -
    //    190.18269 * Math.pow(distanceToHub, 2) +
     //   327.0665 * distanceToHub +
     //   3585.34004
    //return (1400 * distanceToHub + 1166.66667); // 3088.93403;
    SmartDashboard.putString("It deployed", "asdf");
    return (62.77623 * Math.pow(distanceToHub, 2) + 413.13516 * (distanceToHub) + 2754.92691);
  }

  public void climb(double speed) {
   double posistion = climberMotor.getEncoder().getPosition();
    if (speed > 0.01 && posistion > 210) {
      climberMotor.set(0);
    } else if (speed < -0.01 && posistion < -3000) {
    climberMotor.set(0);
    } else {
      climberMotor.set(speed);
    }
    SmartDashboard.putNumber("Climber Posistion", posistion);
  }

  public void climberReset() {
    climberMotor.set(-0.3);
  }


  public void setLaunchSpeed() {
    var rpm = SmartDashboard.getNumber("launcher RPM", 50.0 * 60.0);
    if (rpm <= 0) {
      rpm = calculateRPM(distanceToHubSupplier.get());
      SmartDashboard.putNumber("Target RPM", rpm);
    }
    launcher.setControl(launcherVelocityRequest.withVelocity(rpm / 60.0));
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intake.setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intake.setVoltage(-1 * SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    SmartDashboard.putString("Command", "Launch");
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intake.setVoltage(SmartDashboard.getNumber("Launching intake roller value", LAUNCHING_INTAKE_VOLTAGE));
    setLaunchSpeed();
  }

  // A method to stop the rollers
  public void stop() {
    SmartDashboard.putString("Command", "Stop");
    feederRoller.set(0);
    launcher.setControl(launcherVelocityRequest.withVelocity(18));
    intake.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    SmartDashboard.putString("Command", "Spinup");
    intake.setVoltage(0);
    feederRoller.setVoltage(0);
    setLaunchSpeed();
  }


  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    SmartDashboard.putString("Command", "SpinUp");
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    SmartDashboard.putString("Command", "Launch");
    return this.run(() -> launch());
  }

  public Command autoShootRoutineCommand() {
    return spinUpCommand().withTimeout(1)
        .andThen(launchCommand().withTimeout(10))
        .andThen(runOnce(() -> stop()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("actual launcher RPM", 60 * launcher.getVelocity().getValue().magnitude());
  }
}
