package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestFuelSubsystem extends SubsystemBase implements FuelSubsystem {

    @Override
    public Command spinUpCommand() {
        return runOnce(() -> {});
    }

    @Override
    public Command launchCommand() {
        return runOnce(() -> {});
    }

    @Override public void stop() {}
    @Override public void intake() {}
    @Override public void eject() {}


}
