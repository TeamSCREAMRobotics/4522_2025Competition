package frc2025.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleConsumer;

public class TimedCommand extends Command {

  private double startTime = 0;
  private final double limit;

  private double time = 0;

  private final DoubleConsumer timedFunction;

  public TimedCommand(DoubleConsumer timedFunction, double limit) {
    this.timedFunction = timedFunction;
    this.limit = limit;
  }

  public TimedCommand(DoubleConsumer timedFunction, double limit, Subsystem... requirements) {
    this(timedFunction, limit);
    addRequirements(requirements);
  }

  public TimedCommand(DoubleConsumer timedFunction) {
    this(timedFunction, 0.0);
  }

  public TimedCommand(DoubleConsumer timedFunction, Subsystem... requirements) {
    this(timedFunction);
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    time = startTime;
  }

  @Override
  public void execute() {
    time = Timer.getFPGATimestamp() - startTime;
    timedFunction.accept(time);
  }

  @Override
  public boolean isFinished() {
    if (limit != 0.0) {
      return time >= limit;
    } else {
      return false;
    }
  }
}
