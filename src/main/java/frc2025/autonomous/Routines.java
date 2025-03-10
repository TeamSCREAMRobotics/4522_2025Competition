package frc2025.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2025.RobotContainer;
import frc2025.autonomous.auto_commands.DriveUntilAtPose;
import frc2025.commands.AutoAlign;
import frc2025.commands.Feed;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard.ScoringLocation;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.function.BiFunction;
import java.util.function.Supplier;
import util.AllianceFlipUtil;
import util.PathSequence;

public class Routines {
  private static PathSequence currentSequence;
  private static double eject_TimeOut = 0.25;
  private static double intakeAlgae_TimeOut = 1.5;
  private static double elevator_Timeout = 0.35;
  private static double align_Timeout = 2.5;

  private static final BiFunction<SuperstructureState, RobotContainer, Supplier<Command>>
      applyTargetStateFactory =
          (state, robotContainer) ->
              () -> robotContainer.getSubsystems().superstructure().applyTargetState(state);

  /*
   * Paths are created using the FMS naming convention (A-L, rotating counter-clockwise from the pov of the driver stations)
   * Algae is labeled 1-6 following the same starting and ending sides as coral (1-6, rotating counter-clockwise from the pov of the driver stations)
   */

  /* Test Auto Sequences */
  private static final PathSequence leave = new PathSequence("Leave");

  /* Processor Starting Side Sequences */
  private static final PathSequence E_D_C_B_1 =
      new PathSequence(
          "Processor-Starting_To_E",
          "E_To_Processor-Station",
          "Processor-Station_To_D",
          "D_To_Processor-Station",
          "Processor-Station_To_C",
          "C_To_Processor-Station",
          "Processor-Station_To_B",
          "B_To_1");
  private static final PathSequence E_B_C_D_2 =
      new PathSequence(
          "Processor-Starting_To_E",
          "E_To_Processor-Station",
          "Processor-Station_To_B",
          "B_To_Processor-Station",
          "Processor-Station_To_C",
          "C_To_Processor-Station",
          "Processor-Station_To_D",
          "D_To_2");
  private static final PathSequence E_D_C =
      new PathSequence(
          "Processor-Starting_To_E",
          "E_To_Processor-Station",
          "Processor-Station_To_D",
          "D_To_Processor-Station",
          "Processor-Station_To_C");
  private static final PathSequence D_D_C_C_2P =
      new PathSequence(
          "Processor-Starting_To_D",
          "D_To_Processor-Station",
          "Processor-Station_To_D",
          "D_To_Processor-Station",
          "Processor-Station_To_C",
          "C_To_Processor-Station",
          "Processor-Station_To_C",
          "C_To_2",
          "2_To_Processor");
  private static final PathSequence E_C_C_D_D_2 =
      new PathSequence(
          "Processor-Starting_To_E",
          "E_To_Processor-Station",
          "Processor-Station_To_C",
          "C_To_Processor-Station",
          "Processor-Station_To_C",
          "C_To_Processor-Station",
          "Processor-Station_To_D",
          "D_To_Processor-Station",
          "Processor-Station_To_D",
          "D_To_2");

  /* Middle Starting Side Sequences */
  /* Initial Move is G */
  private static final PathSequence G_4N =
      new PathSequence("Mid-Starting_To_G", "G_To_4", "4_To_Barge1");
  // private static final PathSequence G_4P = new PathSequence();
  private static final PathSequence G_4N_5N_3N = new PathSequence();
  // private static final PathSequence G_4N_5N_3P = new PathSequence();
  private static final PathSequence G_4N_3N_5N = new PathSequence();
  // private static final PathSequence G_4N_3P_5N = new PathSequence();
  /* Initial Move is H */
  private static final PathSequence H_4N =
      new PathSequence("Mid-Starting_To_H", "H_To_4", "4_To_Barge1");
  // private static final PathSequence H_4P = new PathSequence();
  private static final PathSequence H_4N_5N_3N = new PathSequence();
  // private static final PathSequence H_4N_5N_3P = new PathSequence();
  private static final PathSequence H_4N_3N_5N = new PathSequence();
  // private static final PathSequence H_4N_3P_5N = new PathSequence();

  /* Non-Processor Starting Side Sequences */
  private static final PathSequence J_A_L_K_6 =
      new PathSequence(
          "Non-Processor-Starting_To_J",
          "J_To_Non-Processor-Station",
          "Non-Processor-Station_To_A",
          "A_To_Non-Processor-Station",
          "Non-Processor-Station_To_L",
          "L_To_Non-Processor-Station",
          "Non-Processor-Station_To_K",
          "K_To_6");
  private static final PathSequence J_L_K =
      new PathSequence(
          "Non-Processor-Starting_To_J",
          "J_To_Non-Processor-Station",
          "Non-Processor-Station_To_L",
          "L_To_Non-Processor-Station",
          "Non-Processor-Station_To_K");
  private static final PathSequence J_K_L_A_1 =
      new PathSequence(
          "Non-Processor-Starting_To_J",
          "J_To_Non-Processor-Station",
          "Non-Processor-Station_To_K",
          "K_To_Non-Processor-Station",
          "Non-Processor-Station_To_L",
          "L_To_Non-Processor-Station",
          "Non-Processor-Station_To_A",
          "A_To_1");
  private static final PathSequence K_K_L_L_6N2 =
      new PathSequence(
          "Non-Processor-Starting_To_K",
          "K_To_Non-Processor-Station",
          "Non-Processor-Station_To_K",
          "K_To_Non-Processor-Station",
          "Non-Processor-Station_To_L",
          "L_To_Non-Processor-Station",
          "Non-Processor-Station_To_L",
          "L_To_6",
          "6_To_Barge2");
  private static final PathSequence J_L_L_K_K_6 =
      new PathSequence(
          "Non-Processor-Starting_To_J",
          "J_To_Non-Processor-Station",
          "Non-Processor-Station_To_L",
          "L_To_Non-Processor-Station",
          "Non-Processor-Station_To_L",
          "L_To_Non-Processor-Station",
          "Non-Processor-Station_To_K",
          "K_To_Non-Processor-Station",
          "Non-Processor-Station_To_K",
          "K_To_6");

  public static final PathSequence getCurrentSequence() {
    return currentSequence;
  }

  /* Testing Autos */
  public static Command test(RobotContainer container) {
    currentSequence = null;

    return new SequentialCommandGroup(new Feed(container), new PrintCommand("Done"));
  }

  public static Command leave(RobotContainer container) {
    currentSequence = leave;

    return new SequentialCommandGroup(currentSequence.getStart());
  }

  /* Processor Starting Side Autos */
  public static Command processor_Side_E_D_C(RobotContainer container) {
    currentSequence = E_D_C;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart(),
        setSuperstructure(container, SuperstructureState.REEF_L4)
        .withDeadline(
            new AutoAlign(container, ScoringLocation.LEFT).withTimeout(align_Timeout)
        ),
        container.getRobotState().getScoreCommand().get() // Score on E-L4
            .withTimeout(eject_TimeOut), // TODO - is needed or does scoreCommand end?
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout),
        currentSequence.getNext().withDeadline(new Feed(container)),
        Commands.runOnce(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE), wristRollers),
        // TODO - move to feeder station with deadline of feed command
        currentSequence.getNext(),
        setSuperstructure(container, SuperstructureState.REEF_L4)
        .withDeadline(
            new AutoAlign(container, ScoringLocation.RIGHT).withTimeout(align_Timeout)
        ),
        container.getRobotState().getScoreCommand().get() // Score on D-L4
            .withTimeout(eject_TimeOut), // TODO - is needed or does scoreCommand end?
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout),
        currentSequence.getNext().withDeadline(new Feed(container)),
        Commands.runOnce(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE), wristRollers),
        // TODO - move to feeder station with deadline of feed command
        currentSequence.getNext(),
        setSuperstructure(container, SuperstructureState.REEF_L4)
        .withDeadline(
            new AutoAlign(container, ScoringLocation.LEFT).withTimeout(align_Timeout)
        ),
        container.getRobotState().getScoreCommand().get() // Score on C-L4
            .withTimeout(eject_TimeOut), // TODO - is needed or does scoreCommand end?
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout)
    );
  }

  /* Middle Starting Side Autos */  // TODO : rework
  public static Command middle_Side_G_4N(RobotContainer container) {
    currentSequence = G_4N;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            currentSequence.getStart(), setSuperstructure(container, SuperstructureState.REEF_L4)),
        new AutoAlign(container, ScoringLocation.LEFT).withTimeout(align_Timeout),
        container.getRobotState().getScoreCommand().get()
            .withTimeout(eject_TimeOut), // Score pre-load G-L4
        currentSequence.getNext(), // Move to algae
        new ParallelRaceGroup(
            setSuperstructure(container, SuperstructureState.REEF_ALGAE_L1)
                .withTimeout(intakeAlgae_TimeOut),
            wristRollers
                .applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)
                .withTimeout(1.5)
                .andThen(wristRollers.applyGoalCommand(WristRollersGoal.IDLE_AUTO)) // Grab algae 4
            ),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout),
        currentSequence.getNext(),
        setSuperstructure(container, SuperstructureState.BARGE_NET).withTimeout(1.5),
        new DriveUntilAtPose(
            AllianceFlipUtil.get(FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
            container),
        wristRollers.applyGoalCommand(WristRollersGoal.EJECT_ALGAE), // Score algae 4
        setSuperstructure(container, SuperstructureState.FEEDING));
  }

  public static Command middle_Side_H_4N(RobotContainer container) {
    currentSequence = H_4N;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            currentSequence.getStart(), setSuperstructure(container, SuperstructureState.REEF_L4)),
        new AutoAlign(container, ScoringLocation.LEFT).withTimeout(align_Timeout),
        container.getRobotState().getScoreCommand().get()
            .withTimeout(eject_TimeOut), // Score pre-load G-L4
        currentSequence.getNext(), // Move to algae
        new ParallelRaceGroup(
            setSuperstructure(container, SuperstructureState.REEF_ALGAE_L1)
                .withTimeout(intakeAlgae_TimeOut),
            wristRollers
                .applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)
                .withTimeout(1.5)
                .andThen(wristRollers.applyGoalCommand(WristRollersGoal.IDLE_AUTO)) // Grab algae 4
            ),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout),
        currentSequence.getNext(),
        setSuperstructure(container, SuperstructureState.BARGE_NET).withTimeout(1.5),
        new DriveUntilAtPose(
            AllianceFlipUtil.get(FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
            container),
        wristRollers.applyGoalCommand(WristRollersGoal.EJECT_ALGAE), // Score algae 4
        setSuperstructure(container, SuperstructureState.FEEDING));
  }

  /* Non-Processor Starting Side Autos */
  public static Command nonProcessor_Side_J_L_K(RobotContainer container) {
    currentSequence = J_L_K;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart(),
        setSuperstructure(container, SuperstructureState.REEF_L4)
        .withDeadline(
            new AutoAlign(container, ScoringLocation.RIGHT).withTimeout(align_Timeout)
        ),
        container.getRobotState().getScoreCommand().get() // Score on J-L4
            .withTimeout(eject_TimeOut), // TODO - is needed or does scoreCommand end?
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout),
        currentSequence.getNext().withDeadline(new Feed(container)),
        Commands.runOnce(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE), wristRollers),
        // TODO - move to feeder station with deadline of feed command
        currentSequence.getNext(),
        setSuperstructure(container, SuperstructureState.REEF_L4)
        .withDeadline(
            new AutoAlign(container, ScoringLocation.RIGHT).withTimeout(align_Timeout)
        ),
        container.getRobotState().getScoreCommand().get() // Score on L-L4
            .withTimeout(eject_TimeOut), // TODO - is needed or does scoreCommand end?
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout),
        currentSequence.getNext().withDeadline(new Feed(container)),
        Commands.runOnce(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE), wristRollers),
        // TODO - move to feeder station with deadline of feed command
        currentSequence.getNext(),
        setSuperstructure(container, SuperstructureState.REEF_L4)
        .withDeadline(
            new AutoAlign(container, ScoringLocation.LEFT).withTimeout(align_Timeout)
        ),
        container.getRobotState().getScoreCommand().get() // Score on K-L4
            .withTimeout(eject_TimeOut), // TODO - is needed or does scoreCommand end?
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout)
    );
  }

  /* Helper Commands */
  public static final Command setSuperstructure(RobotContainer container, SuperstructureState goalState) {
    return applyTargetStateFactory.apply(goalState, container).get();
  }
}
