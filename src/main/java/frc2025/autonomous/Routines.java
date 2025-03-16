package frc2025.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2025.RobotContainer;
import frc2025.autonomous.auto_commands.DriveUntilAtPose;
import frc2025.commands.AutoAlign;
import frc2025.commands.Feed;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard.ScoringLocation;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.function.BiFunction;
import java.util.function.Supplier;

import drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import util.AllianceFlipUtil;
import util.PathSequence;

public class Routines {
  private static PathSequence currentSequence;
  private static double coralEject_TimeOut = 0.1;
  private static double algaeEject_TimeOut = 0.15;
  private static double intakeAlgae_TimeOut = 1.5;
  private static double idle_Timeout = 0.15;
  private static double elevator_Timeout = 0.20;
  private static double align_Timeout = 3.5;

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
  private static final PathSequence G_4N_5N =
      new PathSequence("Mid-Starting_To_G", "G_To_4", "4_To_Barge1", "Barge1_To_5", "5_To_Barge1");
  // private static final PathSequence G_4P = new PathSequence();
  private static final PathSequence G_4N_5N_3N = new PathSequence();
  // private static final PathSequence G_4N_5N_3P = new PathSequence();
  private static final PathSequence G_4N_3N_5N = new PathSequence();
  // private static final PathSequence G_4N_3P_5N = new PathSequence();
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
  private static final PathSequence J_K_L =
      new PathSequence(
          "Non-Processor-Starting_To_J",
          "J_To_Non-Processor-Station",
          "Non-Processor-Station_To_K",
          "K_To_Non-Processor-Station",
          "Non-Processor-Station_To_L");
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
    currentSequence = leave;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart()
    );
  }

  public static Command leave(RobotContainer container) {
    currentSequence = leave;

    return new SequentialCommandGroup(currentSequence.getStart());
  }

  /* Processor Starting Side Autos */
  public static Command processor_Side_E_D_C(RobotContainer container) {
    currentSequence = E_D_C;
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart().raceWith(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                setSuperstructure(container, SuperstructureState.REEF_L4)
            )
        ).andThen(
            setSuperstructure(container, SuperstructureState.REEF_L4)
            .until(
                () -> elevator.atGoal()
            )
            .alongWith(
                new AutoAlign(container, ScoringLocation.LEFT).withTimeout(align_Timeout)
            )
        ),
        container.getRobotState().getScoreCommand().get() // Score on E-L4
            .withTimeout(coralEject_TimeOut),
        setSuperstructure(container, SuperstructureState.FEEDING).withDeadline(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED),
                    currentSequence.getNext().raceWith(
                        new Feed(container)
                    )
            )
        ),
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                currentSequence.getNext().raceWith(
                    new SequentialCommandGroup(
                        new WaitCommand(1.0),
                        new WaitUntilCommand(() -> WristRollers.hasCoral),
                        setSuperstructure(container, SuperstructureState.REEF_L4)
                    )
                ).andThen(
                    new WaitUntilCommand(() -> WristRollers.hasCoral),
                    setSuperstructure(container, SuperstructureState.REEF_L4)
                    .until(
                        () -> elevator.atGoal()
                    )
                    .alongWith(
                        new AutoAlign(container, ScoringLocation.RIGHT).withTimeout(align_Timeout)
                    )
                )
            ),
            new Feed(container).andThen(
                wristRollers.applyGoalCommand(WristRollersGoal.IDLE)
            )
        ),
        container.getRobotState().getScoreCommand().get() // Score on D-L4
            .withTimeout(coralEject_TimeOut),
        setSuperstructure(container, SuperstructureState.FEEDING).withDeadline(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED),
                    currentSequence.getNext().raceWith(
                        new Feed(container)
                    )
            )
        ),
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                currentSequence.getNext().raceWith(
                    new SequentialCommandGroup(
                        new WaitCommand(1.0),
                        new WaitUntilCommand(() -> WristRollers.hasCoral),
                        setSuperstructure(container, SuperstructureState.REEF_L4)
                    )
                ).andThen(
                    new WaitUntilCommand(() -> WristRollers.hasCoral),
                    setSuperstructure(container, SuperstructureState.REEF_L4)
                    .until(
                        () -> elevator.atGoal()
                    )
                    .alongWith(
                        new AutoAlign(container, ScoringLocation.LEFT).withTimeout(align_Timeout)
                    )
                )
            ),
            new Feed(container).andThen(
                wristRollers.applyGoalCommand(WristRollersGoal.IDLE)
            )
        ),
        container.getRobotState().getScoreCommand().get() // Score on C-L4
            .withTimeout(coralEject_TimeOut),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout)
    );
  }

  /* Middle Starting Side Autos */
  public static Command middle_Side_G_4N(RobotContainer container) {
    currentSequence = G_4N_5N; // Only uses paths for G_4N
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart().raceWith(
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                setSuperstructure(container, SuperstructureState.REEF_L4)
            )
        ).andThen(
            setSuperstructure(container, SuperstructureState.REEF_L4)
            .until(
                () -> elevator.atGoal()
            )
            .alongWith(
                new AutoAlign(container, ScoringLocation.LEFT).withTimeout(align_Timeout)
            )
        ),
        container.getRobotState().getScoreCommand().get() // Score on G-L4
            .withTimeout(coralEject_TimeOut),
        currentSequence.getNext(),
        new AutoAlign(container, ScoringLocation.CENTER).alongWith(
            setSuperstructure(container, SuperstructureState.REEF_ALGAE_L1).withDeadline(
                wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE).withTimeout(intakeAlgae_TimeOut)
            ).andThen(
                wristRollers.applyGoalCommand(WristRollersGoal.IDLE_AUTO).until(
                    () -> wristRollers.getGoal() == (TalonFXSubsystemGoal) WristRollersGoal.IDLE_AUTO
                )
            )
        ),
        setSuperstructure(container, SuperstructureState.FEEDING).raceWith(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED),
                currentSequence.getNext()
            )
        ),
        setSuperstructure(container, SuperstructureState.BARGE_NET).until(
            () -> elevator.atGoal()
        ),
        new DriveUntilAtPose(
            AllianceFlipUtil.get(
                FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
        container),
        wristRollers.applyGoalCommand(WristRollersGoal.EJECT_ALGAE).withTimeout(algaeEject_TimeOut), // Score algae 4
        setSuperstructure(container, SuperstructureState.FEEDING).until(
            () -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED
        )
    );
  }

  public static Command middle_Side_H_4N(RobotContainer container) {
    currentSequence = H_4N;
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart().raceWith(
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                setSuperstructure(container, SuperstructureState.REEF_L4)
            )
        ).andThen(
            setSuperstructure(container, SuperstructureState.REEF_L4)
            .until(
                () -> elevator.atGoal()
            )
            .alongWith(
                new AutoAlign(container, ScoringLocation.RIGHT).withTimeout(align_Timeout)
            )
        ),
        container.getRobotState().getScoreCommand().get() // Score on H-L4
            .withTimeout(coralEject_TimeOut),
        currentSequence.getNext(),
        new AutoAlign(container, ScoringLocation.CENTER).alongWith(
            setSuperstructure(container, SuperstructureState.REEF_ALGAE_L1).withDeadline(
                wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE).withTimeout(intakeAlgae_TimeOut)
            ).andThen(
                wristRollers.applyGoalCommand(WristRollersGoal.IDLE_AUTO).until(
                    () -> wristRollers.getGoal() == (TalonFXSubsystemGoal) WristRollersGoal.IDLE_AUTO
                )
            )
        ),
        setSuperstructure(container, SuperstructureState.FEEDING).raceWith(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED),
                currentSequence.getNext()
            )
        ),
        setSuperstructure(container, SuperstructureState.BARGE_NET).until(
            () -> elevator.atGoal()
        ),
        new DriveUntilAtPose(
            AllianceFlipUtil.get(
                FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
        container),
        wristRollers.applyGoalCommand(WristRollersGoal.EJECT_ALGAE).withTimeout(algaeEject_TimeOut), // Score algae 4
        setSuperstructure(container, SuperstructureState.FEEDING).until(
            () -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED
        )
    );
  }

  public static Command middle_Side_G_4N_5N(RobotContainer container) {
    return new SequentialCommandGroup(
        middle_Side_G_4N(container),
        new WaitCommand(10.0),
        middle_Side_5N(container)
    );
  }

  public static Command middle_Side_H_4N_5N(RobotContainer container) {
    return new SequentialCommandGroup(
        // middle_Side_H_4N(container),
        // middle_Side_5N(container)
    );
  }

  public static Command middle_Side_5N(RobotContainer container) {
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();

    return new SequentialCommandGroup(
        currentSequence.getNext(), // Run to algae 5
        new AutoAlign(container, ScoringLocation.CENTER).alongWith(
            setSuperstructure(container, SuperstructureState.REEF_ALGAE_L2).withDeadline(
                wristRollers.applyGoalCommand(WristRollersGoal.INTAKE_ALGAE).withTimeout(intakeAlgae_TimeOut)
            ).andThen(
                wristRollers.applyGoalCommand(WristRollersGoal.IDLE_AUTO).withTimeout(idle_Timeout)
            )
        ),
        setSuperstructure(container, SuperstructureState.FEEDING).raceWith(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED),
                currentSequence.getNext()
            )
        ),
        setSuperstructure(container, SuperstructureState.BARGE_NET).until(
            () -> elevator.atGoal()
        ),
        new DriveUntilAtPose(
            AllianceFlipUtil.get(
                FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
        container),
        wristRollers.applyGoalCommand(WristRollersGoal.EJECT_ALGAE).withTimeout(algaeEject_TimeOut), // Score algae 4
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout),
        new WaitUntilCommand(() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED)
    );
  }

  /* Non-Processor Starting Side Autos */
  public static Command nonProcessor_Side_J_K_L(RobotContainer container) {
    currentSequence = J_K_L;
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart().raceWith(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                setSuperstructure(container, SuperstructureState.REEF_L4)
            )
        ).andThen(
            setSuperstructure(container, SuperstructureState.REEF_L4)
            .until(
                () -> elevator.atGoal()
            )
            .alongWith(
                new AutoAlign(container, ScoringLocation.RIGHT).withTimeout(align_Timeout)
            )
        ),
        container.getRobotState().getScoreCommand().get() // Score on J-L4
            .withTimeout(coralEject_TimeOut),
        setSuperstructure(container, SuperstructureState.FEEDING).withDeadline(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED),
                    currentSequence.getNext().raceWith(
                        new Feed(container)
                    )
            )
        ),
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                currentSequence.getNext().raceWith(
                    new SequentialCommandGroup(
                        new WaitCommand(1.0),
                        new WaitUntilCommand(() -> WristRollers.hasCoral),
                        setSuperstructure(container, SuperstructureState.REEF_L4)
                    )
                ).andThen(
                    new WaitUntilCommand(() -> WristRollers.hasCoral),
                    setSuperstructure(container, SuperstructureState.REEF_L4)
                    .until(
                        () -> elevator.atGoal()
                    )
                    .alongWith(
                        new AutoAlign(container, ScoringLocation.LEFT).withTimeout(align_Timeout)
                    )
                )
            ),
            new Feed(container).andThen(
                wristRollers.applyGoalCommand(WristRollersGoal.IDLE)
            )
        ),
        container.getRobotState().getScoreCommand().get() // Score on K-L4
            .withTimeout(coralEject_TimeOut),
        setSuperstructure(container, SuperstructureState.FEEDING).withDeadline(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED),
                    currentSequence.getNext().raceWith(
                        new Feed(container)
                    )
            )
        ),
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                currentSequence.getNext().raceWith(
                    new SequentialCommandGroup(
                        new WaitCommand(1.0),
                        new WaitUntilCommand(() -> WristRollers.hasCoral),
                        setSuperstructure(container, SuperstructureState.REEF_L4)
                    )
                ).andThen(
                    new WaitUntilCommand(() -> WristRollers.hasCoral),
                    setSuperstructure(container, SuperstructureState.REEF_L4)
                    .until(
                        () -> elevator.atGoal()
                    )
                    .alongWith(
                        new AutoAlign(container, ScoringLocation.RIGHT).withTimeout(align_Timeout)
                    )
                )
            ),
            new Feed(container).andThen(
                wristRollers.applyGoalCommand(WristRollersGoal.IDLE)
            )
        ),
        container.getRobotState().getScoreCommand().get() // Score on L-L4
            .withTimeout(coralEject_TimeOut),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout)
    );
  }

  /* Helper Commands */
  public static final Command setSuperstructure(RobotContainer container, SuperstructureState goalState) {
    return applyTargetStateFactory.apply(goalState, container).get();
  }
}
