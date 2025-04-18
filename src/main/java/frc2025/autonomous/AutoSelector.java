package frc2025.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.RobotContainer;
import java.util.function.Function;

public class AutoSelector {

  public enum AutoMode {
    DO_NOTHING(container -> Commands.none()),
    TEST(container -> Routines.test(container)),
    LEAVE(container -> Routines.leave(container)),
    // Oklahoma Autos
    // PROCESSOR_EDC(container -> Routines.processor_Side_E_D_C(container)),
    // NONPROCESSOR_JKL(container -> Routines.nonProcessor_Side_J_K_L(container)),
    /* MID_G_4N(container -> Routines.middle_Side_G_4N(container)),
    MID_H_4N(container -> Routines.middle_Side_H_4N(container)),
    PROCESSOR_PUSH(container -> Routines.processor_Push(container)),
    NONPROCESSOR_PUSH(container -> Routines.processor_Push(container)), */
    PROCESSOR_E_B_C_D(container -> Routines.processor_E_B_C_D(container)),
    NONPROCESSOR_J_A_L_K(container -> Routines.nonProcessor_J_A_L_K(container)),
    PROCESSOR_E_C_D_2(container -> Routines.processor_E_C_D_2(container)),
    NONPROCESSOR_J_L_K_6(container -> Routines.nonProcessor_J_L_K_6(container)),
    // MID_G_E_F(container -> Routines.mid_G_E_F(container)),
    MID_G_4_5(container -> Routines.mid_G_4_5(container, false)),
    FLINT_N_STEAL(container -> Routines.mid_G_4_5(container, true));

    private final Function<RobotContainer, Command> factory;

    private AutoMode(Function<RobotContainer, Command> factory) {
      this.factory = factory;
    }

    public Command getCommand(RobotContainer container) {
      return factory.apply(container);
    }
  }

  private final RobotContainer container;

  private final SendableChooser<AutoMode> chooser = new SendableChooser<>();

  public AutoSelector(RobotContainer container) {
    this.container = container;

    for (AutoMode mode : AutoMode.values()) {
      chooser.addOption(mode.name(), mode);
    }
    chooser.setDefaultOption(AutoMode.DO_NOTHING.name(), AutoMode.DO_NOTHING);
    SmartDashboard.putData("AutoSelector", chooser);
  }

  public AutoMode getSelected() {
    return chooser.getSelected();
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected().getCommand(container);
  }
}
