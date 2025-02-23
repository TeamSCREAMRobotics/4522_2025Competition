package frc2025.subsystems.superstructure.wrist;

import com.ctre.phoenix6.hardware.CANrange;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.util.Units;
import frc2025.logging.Logger;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class WristRollers extends TalonFXSubsystem {

  private final CANrange beam = new CANrange(0);
  public static boolean hasGamePiece = false;

  public WristRollers(TalonFXSubsystemConfiguration config) {
    super(config, WristRollersGoal.IDLE);

    beam.getDistance().setUpdateFrequency(300.0);
  }

  public enum WristRollersGoal implements TalonFXSubsystemGoal {
    IDLE(() -> hasGamePiece ? 0.0 : 1.0, ControlType.VOLTAGE),
    INTAKE(() -> 7.5, ControlType.VOLTAGE),
    EJECT_CORAL(() -> 12.0, ControlType.VOLTAGE),
    EJECT_ALGAE(() -> -10.0, ControlType.VOLTAGE);

    public final DoubleSupplier voltage;
    public final ControlType controlType;

    private WristRollersGoal(DoubleSupplier voltage, ControlType controlType) {
      this.voltage = voltage;
      this.controlType = controlType;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }

    @Override
    public DoubleSupplier target() {
      return voltage;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }
  }

  private double getBeamDistanceInches() {
    return Units.metersToInches(beam.getDistance().getValueAsDouble());
  }

  public BooleanSupplier acquiredGamePiece() {
    return () -> getBeamDistanceInches() < 3.0;
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "HasGamePiece", acquiredGamePiece().getAsBoolean());
    Logger.log(logPrefix + "BeamDistance", getBeamDistanceInches());
  }
}
