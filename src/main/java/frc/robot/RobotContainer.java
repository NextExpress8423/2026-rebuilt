// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public enum HangPosition {
        NO_HANG,
        HANG_LEFT,
        HANG_RIGHT
    }

    // The robot's subsystems
    private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
    private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem(driveSubsystem.getHubDistanceSupplier());

    // The driver's controller
    private final CommandXboxController driverController = new CommandXboxController(
            DRIVER_CONTROLLER_PORT);

    private final SlewRateLimiter fowardSlewRateLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter trueningSlewRateLimiter = new SlewRateLimiter(4);
    // The operator's controller
    private final CommandXboxController operatorController = new CommandXboxController(
            OPERATOR_CONTROLLER_PORT);

    // The autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SendableChooser<HangPosition> autoHangSelection = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();

        // Set the options to show up in the Dashboard for selecting auto modes. If you
        // add additional auto modes you can add additional lines here with
        // autoChooser.addOption

        autoHangSelection.setDefaultOption(HangPosition.NO_HANG.toString(), HangPosition.NO_HANG);
        autoHangSelection.addOption(HangPosition.HANG_LEFT.toString(), HangPosition.HANG_LEFT);
        autoHangSelection.addOption(HangPosition.HANG_RIGHT.toString(), HangPosition.HANG_RIGHT);
        SmartDashboard.putData("AutoHangPosition", autoHangSelection);
        Autos.hangPositionChooser = autoHangSelection;

        autoChooser.addOption("BLUE Hub Auto", Autos.hubAuto(driveSubsystem, ballSubsystem, Alliance.Blue));
        autoChooser.addOption("BLUE Right Trench Auto", Autos.rightTrench(driveSubsystem, ballSubsystem, Alliance.Blue));
        autoChooser.addOption("BLUE Left Trench Auto", Autos.leftTrench(driveSubsystem, ballSubsystem, Alliance.Blue));
        autoChooser.addOption("RED Hub Auto", Autos.hubAuto(driveSubsystem, ballSubsystem, Alliance.Red));
        autoChooser.addOption("RED Right Trench Auto", Autos.rightTrench(driveSubsystem, ballSubsystem, Alliance.Red));
        autoChooser.addOption("RED Left Trench Auto", Autos.leftTrench(driveSubsystem, ballSubsystem, Alliance.Red));
        autoChooser.addOption("Blue Left Hide Away Auto", Autos.hideAwayAuto(driveSubsystem, ballSubsystem, Alliance.Blue));
        autoChooser.addOption("Blue Left Trench Depo Auto", Autos.depoAuto(driveSubsystem, ballSubsystem, Alliance.Blue));
     //   autoChooser.addOption("Drive Foward Four Meters", Autos.driveFowardFourMeters(driveSubsystem, ballSubsystem));
      //  autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem, ballSubsystem));
        SmartDashboard.putData("Autos", autoChooser);

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for {@link CommandXboxController Xbox}/
     * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        // While the left bumper on operator controller is held, intake Fuel
        operatorController.leftBumper()
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
        // While the right bumper on the operator controller is held, spin up for 1
        // second, then launch fuel. When the button is released, stop.
        operatorController.rightBumper()
                .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
                        .andThen(ballSubsystem.launchCommand())
                        .finallyDo(() -> ballSubsystem.stop()));
        // While the A button is held on the operator controller, eject fuel back out
        // the intake
        operatorController.a()
                .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));

        // Set the default command for the drive subsystem to the command provided by
        // factory with the values provided by the joystick axes on the driver
        // controller. The Y axis of the controller is inverted so that pushing the
        // stick away from you (a negative value) drives the robot forwards (a positive
        // value). The X-axis is also inverted so a positive value (stick to the right)
        // results in clockwise rotation (front of the robot turning right). Both axes
        // are also scaled down so the rotation is more easily controllable. 
        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveArcade(
                        () -> fowardSlewRateLimiter.calculate(-driverController.getLeftY()) * DRIVE_SCALING,
                        () -> trueningSlewRateLimiter.calculate(-driverController.getRightX()) * ROTATION_SCALING));

        driverController.a()
                .whileTrue(driveSubsystem.turnToHubCommand());

        driverController.pov(0).whileTrue(Commands.run(() -> ballSubsystem.climb(1))).whileFalse(Commands.run(() -> ballSubsystem.climb(0)));
        driverController.pov(180).whileTrue(Commands.run(() -> ballSubsystem.climb(-1))).whileFalse(Commands.run(() -> ballSubsystem.climb(0)));

        // resets infront of the hub, same starting spot for our hub autos.
        driverController.b()
                //Add red check things n' such here
                .onTrue(driveSubsystem.manualPoseResetCommand());

        driverController.y()
                .whileTrue(driveSubsystem.shakeThingsUpCommand());

        driverController.rightBumper()
                .whileTrue(Commands.startEnd(
                        () -> {
                                DRIVE_SCALING = 0.85;
                                ROTATION_SCALING = 0.7;
                        }, 
                        () -> {
                        DRIVE_SCALING = 0.75;
                        ROTATION_SCALING = 0.6;
                        }));
                        
        driverController.leftBumper()
                .whileTrue(Commands.startEnd(
                        () -> {
                                DRIVE_SCALING = 0.6;
                                ROTATION_SCALING = 0.5;
                        }, 
                        () -> {
                        DRIVE_SCALING = 0.75;
                        ROTATION_SCALING = 0.6;
                        }));


        // driverController.rightBumper()
        //         .and(driverController.a())
        //         .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        // driverController.rightBumper()
        //         .and(driverController.b())
        //         .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // driverController.rightBumper()
        //         .and(driverController.x())
        //         .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

        // driverController.rightBumper()
        //         .and(driverController.y())
        //         .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
}
