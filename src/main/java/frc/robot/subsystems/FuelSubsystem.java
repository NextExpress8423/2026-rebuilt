package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public interface FuelSubsystem {
    Command spinUpCommand();
    Command launchCommand();
    void stop();
    void intake();
    void eject();
}
