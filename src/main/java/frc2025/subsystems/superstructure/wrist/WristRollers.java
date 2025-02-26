package frc2025.subsystems.superstructure.wrist;

import com.ctre.phoenix6.hardware.CANrange;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.Robot;
import frc2025.logging.Logger;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Setter;

public class WristRollers extends TalonFXSubsystem {

  private final CANrange beam = new CANrange(0);
  public static boolean hasGamePiece = false;

  @Setter private static double idleVoltage = 1.0;

  private final Debouncer beamDebouncer = new Debouncer(0.025);

  public WristRollers(TalonFXSubsystemConfiguration config) {
    super(config, WristRollersGoal.IDLE);

    beam.getDistance().setUpdateFrequency(300.0);
  }

  public enum WristRollersGoal implements TalonFXSubsystemGoal {
    IDLE(() -> hasGamePiece ? 0.0 : -1.0, ControlType.VOLTAGE),
    HOLD(() -> 0.0, ControlType.VOLTAGE),
    INTAKE(() -> 9.0, ControlType.VOLTAGE),
    INTAKE_ALGAE(() -> -9.0, ControlType.VOLTAGE),
    INTAKE_TROUGH(() -> -9.0, ControlType.VOLTAGE),
    EJECT_CORAL(() -> 12.0, ControlType.VOLTAGE),
    EJECT_ALGAE(() -> 12.0, ControlType.VOLTAGE);

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

  public boolean acquiredGamePiece() {
    return getBeamDistanceInches() < 2.0;
  }

  public BooleanSupplier hasGamePiece() {
    if (Robot.isSimulation()) {
      return () -> false;
    } else {
      return () -> beamDebouncer.calculate(acquiredGamePiece());
    }
  }

  public static void resetBeam() {
    hasGamePiece = false;
    idleVoltage = 1.0;
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "HasGamePiece", hasGamePiece);
    Logger.log(logPrefix + "AcquiredGamePiece", acquiredGamePiece());
    Logger.log(logPrefix + "BeamDistance", getBeamDistanceInches());
  }
}
