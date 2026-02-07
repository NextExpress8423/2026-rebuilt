package frc.robot.commands;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class DynamicCommand<T> extends Command {
    private Supplier<T> optionSupplier;
    private Map<T, Command> options = new HashMap<>();
    private Command delegate;

    public DynamicCommand(Supplier<T> optionSupplier) {
        this.optionSupplier = optionSupplier;
    }

    public DynamicCommand<T> withOption(T value, Command command) {
        options.put(value, command);
        return this;
    }

    public void initialize() {
        delegate = options.get(optionSupplier.get());
        delegate.initialize();
    }

    public void execute() {
        delegate.execute();
    }

    public void end(boolean interrupted) {
        delegate.end(interrupted);
    }

    public boolean isFinished() {
        return delegate.isFinished();
    }
}
