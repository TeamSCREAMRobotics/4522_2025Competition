package frc2025.subsystems.intake;

import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;

public class IntakeDeploy extends TalonFXSubsystem {

  public IntakeDeploy(TalonFXSubsystemConfiguration config) {
    super(config, IntakeDeployGoal.IDLE);
  }

  public enum IntakeDeployGoal implements TalonFXSubsystemGoal {
    HOME(Rotation2d.fromDegrees(0)),
    IDLE(Rotation2d.fromDegrees(90)),
    DEPLOY(Rotation2d.fromDegrees(145));

    public final double targetRotations;

    private IntakeDeployGoal(Rotation2d angle) {
      this.targetRotations = angle.getRotations();
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier target() {
      return () -> targetRotations;
    }
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "Angle", getAngle());
  }
}
