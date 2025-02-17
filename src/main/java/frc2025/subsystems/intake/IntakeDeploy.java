package frc2025.subsystems.intake;

import data.Length;
import drivers.TalonFXSubsystem;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;

public class IntakeDeploy extends TalonFXSubsystem {

  public IntakeDeploy(TalonFXSubsystemConfiguration config) {
    super(config, IntakeDeployGoal.HOME);
  }

  public enum IntakeDeployGoal implements TalonFXSubsystemGoal {
    HOME(Length.kZero),
    HOLDING(Length.fromInches(10.0)),
    MAX(IntakeConstants.MAX_EXTENSION);

    public final double targetRotations;

    private IntakeDeployGoal(Length extension) {
      this.targetRotations =
          extension.getInches() / IntakeConstants.PINION_CIRCUMFERENCE.getInches();
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier target() {
      return () -> targetRotations;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }
  }

  public Length getMeasuredExtension() {
    return Length.fromRotations(getPosition(), IntakeConstants.PINION_CIRCUMFERENCE);
  }

  public Length getSetpointExtension() {
    return Length.fromRotations(getSetpoint(), IntakeConstants.PINION_CIRCUMFERENCE);
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "MeasuredExtension", getMeasuredExtension().getInches());
    Logger.log(logPrefix + "SetpointExtension", getMeasuredExtension().getInches());
  }
}
