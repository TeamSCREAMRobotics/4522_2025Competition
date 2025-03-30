package frc2025.subsystems.superstructure.wrist;

import com.ctre.phoenix6.hardware.CANrange;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc2025.Dashboard;
import frc2025.Robot;
import frc2025.logging.Logger;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class WristRollers extends TalonFXSubsystem {

  private static final CANrange beam = new CANrange(0);
  public static boolean hasCoral = true;

  private final Debouncer beamDebouncer = new Debouncer(0.1); // 0.0875

  public WristRollers(TalonFXSubsystemConfiguration config) {
    super(config);

    beam.getDistance().setUpdateFrequency(300.0);
  }

  public enum WristRollersGoal implements TalonFXSubsystemGoal {
    IDLE(() -> acquiredCoral() ? 1.0 : -1.0, ControlType.VOLTAGE),
    IDLE_AUTO(() -> -2.0, ControlType.VOLTAGE),
    HOLD(() -> 0.0, ControlType.VOLTAGE),
    INTAKE(() -> 7.5, ControlType.VOLTAGE),
    INTAKE_ALGAE(() -> -12.0, ControlType.VOLTAGE),
    INTAKE_TROUGH(() -> -9.0, ControlType.VOLTAGE),
    EJECT_CORAL(() -> 12.0, ControlType.VOLTAGE),
    EJECT_ALGAE(() -> 9.0, ControlType.VOLTAGE),
    TROUGH(() -> 8.0, ControlType.VOLTAGE);

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

  private static double getBeamDistanceInches() {
    return Units.metersToInches(beam.getDistance().getValueAsDouble());
  }

  public static boolean acquiredCoral() {
    return getBeamDistanceInches() < 2.25;
  }

  public BooleanSupplier hasCoral() {
    if (Robot.isSimulation()) {
      return () -> Dashboard.Sim.hasCoral.get();
    } else {
      return () -> beamDebouncer.calculate(acquiredCoral());
    }
  }

  public static void resetBeam() {
    if (Robot.isSimulation()) {
      Dashboard.Sim.hasCoral.set(false);
    }
    hasCoral = false;
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "HasCoral", hasCoral);
    Logger.log(logPrefix + "AcquiredCoral", acquiredCoral());
    Logger.log(logPrefix + "BeamDistance", getBeamDistanceInches());
  }
}
