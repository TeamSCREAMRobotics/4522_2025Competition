package frc2025.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2025.RobotContainer;
import frc2025.autonomous.auto_commands.AutoFeed;
import frc2025.autonomous.auto_commands.DriveUntilAtPose;
import frc2025.constants.FieldConstants;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.function.BiFunction;
import java.util.function.Supplier;
import util.PathSequence;

public class Routines {
  private static PathSequence currentSequence;
  private static double eject_TimeOut = 0.25;
  private static double intakeAlgae_TimeOut = 0.25;
  private static double feederStation_TimeOut = 0.5;
  private static double elevatorAfterFeed_TimeOut = 0.5;
  private static double elevator_Timeout = 0.5;

  private static final BiFunction<SuperstructureState, RobotContainer, Supplier<Command>>
      applyTargetStateFactory =
          (state, robotContainer) ->
              () -> robotContainer.getSubsystems().superstructure().applyTargetState(state);

  /*
   * Paths are created using the FMS naming convention (A-L, rotating counter-clockwise from the pov of the driver stations)
   * Algae is labeled 1-6 following the same starting and ending sides as coral (1-6, rotating counter-clockwise from the pov of the driver stations)
   */

  /* Test Auto Sequences */
  private static final PathSequence maxSpeedTest = new PathSequence("MaxSpeed_Test");

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
  private static final PathSequence E_C_D_2 =
      new PathSequence(
          "Processor-Starting_To_E",
          "E_To_Processor-Station",
          "Processor-Station_To_C",
          "C_To_Processor-Station",
          "Processor-Station_To_D",
          "D_To_2");
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
  public static Command maxSpeedTest(RobotContainer container) {
    currentSequence = maxSpeedTest;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        new AutoFeed(container),
        new WaitCommand(5.0),
        new ParallelRaceGroup(
            currentSequence.getStart(), setElevator(container, SuperstructureState.REEF_L4)),
        wristRollers.applyGoalCommand(WristRollersGoal.EJECT_CORAL).withTimeout(eject_TimeOut));
  }

  /* Processor Starting Side Autos */
  public static Command processor_Side_E_C_D_2(RobotContainer container) {
    currentSequence = E_C_D_2;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            currentSequence.getStart(), setElevator(container, SuperstructureState.REEF_L4)),
        // new DriveUntilAtPose(null, container),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score pre-load E-L4
        setElevator(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout),
        new InstantCommand(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        new ParallelCommandGroup(currentSequence.getNext(), new AutoFeed(container)),
        // new WaitCommand(feederStation_TimeOut),
        new ParallelRaceGroup(
            currentSequence.getNext(),
            new SequentialCommandGroup(
                new WaitCommand(elevatorAfterFeed_TimeOut),
                setElevator(container, SuperstructureState.REEF_L4))),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score pre-load C-L4
            setElevator(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout),
        new InstantCommand(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        new ParallelCommandGroup(currentSequence.getNext(), new AutoFeed(container)),
        // new WaitCommand(feederStation_TimeOut),
        new ParallelRaceGroup(
            currentSequence.getNext(),
            new SequentialCommandGroup(
                new WaitCommand(elevatorAfterFeed_TimeOut),
                setElevator(container, SuperstructureState.REEF_L4))),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score pre-load D-L4
        new InstantCommand(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        currentSequence.getNext(), // Move to algae
        // setElevator(container, SuperstructureState.REEF_ALGAE_L1).withTimeout(0.25),
        // wristRollers.applyGoalCommand(WristRollersGoal.INTAKE).withTimeout(intakeAlgae_TimeOut),
        // // Grab algae 2
        setElevator(container, SuperstructureState.FEEDING).withTimeout(elevator_Timeout));
  }

  public static Command processor_Side_E_B_C_D_2(RobotContainer container) {
    currentSequence = E_B_C_D_2;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            currentSequence.getStart(), setElevator(container, SuperstructureState.REEF_L4)),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score pre-load E-L4
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)),
        new InstantCommand(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        new ParallelRaceGroup(currentSequence.getNext(), new AutoFeed(container)),
        new WaitCommand(feederStation_TimeOut),
        new ParallelRaceGroup(
            currentSequence.getNext(), setElevator(container, SuperstructureState.REEF_L4)),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score pre-load B-L4
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)),
        new InstantCommand(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        new ParallelRaceGroup(currentSequence.getNext(), new AutoFeed(container)),
        new WaitCommand(feederStation_TimeOut),
        new ParallelRaceGroup(
            currentSequence.getNext(), setElevator(container, SuperstructureState.REEF_L4)),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score pre-load C-L4
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)),
        new InstantCommand(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        new ParallelRaceGroup(currentSequence.getNext(), new AutoFeed(container)),
        new WaitCommand(feederStation_TimeOut),
        new ParallelRaceGroup(
            currentSequence.getNext(), setElevator(container, SuperstructureState.REEF_L4)),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score D-L4
        new InstantCommand(() -> wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        currentSequence.getNext(), // Move to algae
        // setElevator(container, SuperstructureState.REEF_ALGAE_L1).withTimeout(0.25),
        // wristRollers.applyGoalCommand(WristRollersGoal.INTAKE).withTimeout(intakeAlgae_TimeOut),
        // // Grab algae 2
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)));
  }

  /* Middle Starting Side Autos */
  public static Command middle_Side_G_4N(RobotContainer container) {
    currentSequence = G_4N;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        currentSequence.getIndex(0),
        setElevator(container, SuperstructureState.REEF_L4).withTimeout(elevator_Timeout),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score pre-load G-L4
        currentSequence.getNext(),
        setElevator(container, SuperstructureState.REEF_ALGAE_L2)
            .withTimeout(intakeAlgae_TimeOut), // Grab algae 4
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)),
        currentSequence.getNext(),
        setElevator(container, SuperstructureState.BARGE_NET).withTimeout(elevator_Timeout),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_ALGAE)
            .withTimeout(eject_TimeOut), // Score algae 4 in barge
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)));
  }

  /* Non-Processor Starting Side Autos */
  public static Command nonProcessor_Side_J_A_L_K_6(RobotContainer container) {
    currentSequence = J_A_L_K_6;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            currentSequence.getStart(), setElevator(container, SuperstructureState.REEF_L4)),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score pre-load J-L4
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)),
        new ParallelRaceGroup(currentSequence.getNext(), new AutoFeed(container)),
        new ParallelRaceGroup(
            currentSequence.getNext(), setElevator(container, SuperstructureState.REEF_L4)),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score A-L4
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)),
        new ParallelRaceGroup(currentSequence.getNext(), new AutoFeed(container)),
        new ParallelRaceGroup(
            currentSequence.getNext(), setElevator(container, SuperstructureState.REEF_L4)),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score L-L4
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)),
        new ParallelRaceGroup(currentSequence.getNext(), new AutoFeed(container)),
        new ParallelRaceGroup(
            currentSequence.getNext(), setElevator(container, SuperstructureState.REEF_L4)),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_CORAL)
            .withTimeout(eject_TimeOut), // Score K-L4
        currentSequence.getNext(), // Move to algae
        setElevator(container, SuperstructureState.REEF_ALGAE_L1).withTimeout(elevator_Timeout),
        wristRollers
            .applyGoalCommand(WristRollersGoal.INTAKE)
            .withTimeout(intakeAlgae_TimeOut), // Grab algae 6
        new InstantCommand(() -> setElevator(container, SuperstructureState.HOME)));
  }

  /* Helper Commands */
  public static final Command setElevator(RobotContainer container, SuperstructureState goalState) {
    return applyTargetStateFactory.apply(goalState, container).get();
  }
}
