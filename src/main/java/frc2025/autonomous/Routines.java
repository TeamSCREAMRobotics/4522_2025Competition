package frc2025.autonomous;

import drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2025.RobotContainer;
import frc2025.autonomous.auto_commands.AutoAlign2;
import frc2025.autonomous.auto_commands.AutoScore2;
import frc2025.autonomous.auto_commands.DriveUntilAtPose;
import frc2025.commands.AutoAlign;
import frc2025.commands.AutoScore;
import frc2025.commands.Feed;
import frc2025.constants.FieldConstants;
import frc2025.constants.FieldConstants.ReefLocation;
import frc2025.controlboard.Controlboard.ScoringLocation;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.elevator.Elevator;
import frc2025.subsystems.superstructure.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.superstructure.wrist.Wrist;
import frc2025.subsystems.superstructure.wrist.Wrist.WristGoal;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import java.util.ArrayList;
import java.util.function.BiFunction;
import java.util.function.Supplier;
import util.AllianceFlipUtil;
import util.PathSequence;

public class Routines {
  private static PathSequence currentSequence;
  private static double applyGoal_TimeOut = 0.1;
  private static double elevatorHeightTolerance =
      20.0; // Height (in) the elevator should be under before moving again
  private static double highElevatorHeightTolerance_L4 = 63.0;
  private static double startPath_TimeOut = 0.15;

  private static double coralEject_TimeOut = 0.1;
  private static double algaeEject_TimeOut = 0.15;
  private static double intakeAlgae_TimeOut = 0.35;
  private static double idle_Timeout = 0.25;
  private static double elevator_Timeout = 0.20;
  private static double align_Timeout = 3.5;

  private static double feederAlignTolerance = 0.08;

  private static final BiFunction<SuperstructureState, RobotContainer, Supplier<Command>>
      applyTargetStateFactory =
          (state, robotContainer) ->
              () -> robotContainer.getSubsystems().superstructure().applyTargetState(state);

  private static Pose2d feederAlliance_Pose;
  private static Pose2d bargeAlliance_Pose;
  private static ArrayList<Supplier<ReefLocation>> scoreSequence;
  private static final ArrayList<Supplier<ReefLocation>> processor_E_B_C_D =
      new ArrayList<Supplier<ReefLocation>>();
  private static final ArrayList<Supplier<ReefLocation>> processor_E_C_D_2 =
      new ArrayList<Supplier<ReefLocation>>();
  private static final ArrayList<Supplier<ReefLocation>> nonProcessor_J_A_L_K =
      new ArrayList<Supplier<ReefLocation>>();
    private static final ArrayList<Supplier<ReefLocation>> nonProcessor_J_L_K_6 =
        new ArrayList<Supplier<ReefLocation>>();
  private static final ArrayList<Supplier<ReefLocation>> nonProcessor_J_L_K_K =
      new ArrayList<Supplier<ReefLocation>>();
  private static final ArrayList<Supplier<ReefLocation>> midProcessor_G_E_F =
      new ArrayList<Supplier<ReefLocation>>();
  private static final ArrayList<Supplier<ReefLocation>> midNonProcessor_H_J_I =
      new ArrayList<Supplier<ReefLocation>>();

  static {
    processor_E_B_C_D.add(() -> ReefLocation.E);
    processor_E_B_C_D.add(() -> ReefLocation.B);
    processor_E_B_C_D.add(() -> ReefLocation.C);
    processor_E_B_C_D.add(() -> ReefLocation.D);

    processor_E_C_D_2.add(() -> ReefLocation.E);
    processor_E_C_D_2.add(() -> ReefLocation.C);
    processor_E_C_D_2.add(() -> ReefLocation.D);

    nonProcessor_J_A_L_K.add(() -> ReefLocation.J);
    nonProcessor_J_A_L_K.add(() -> ReefLocation.A);
    nonProcessor_J_A_L_K.add(() -> ReefLocation.L);
    nonProcessor_J_A_L_K.add(() -> ReefLocation.K);

    nonProcessor_J_L_K_6.add(() -> ReefLocation.J);
    nonProcessor_J_L_K_6.add(() -> ReefLocation.L);
    nonProcessor_J_L_K_6.add(() -> ReefLocation.K);

    nonProcessor_J_L_K_K.add(() -> ReefLocation.J);
    nonProcessor_J_L_K_K.add(() -> ReefLocation.L);
    nonProcessor_J_L_K_K.add(() -> ReefLocation.K);
    nonProcessor_J_L_K_K.add(() -> ReefLocation.K);

    midProcessor_G_E_F.add(() -> ReefLocation.G);
    midProcessor_G_E_F.add(() -> ReefLocation.E);
    midProcessor_G_E_F.add(() -> ReefLocation.F);

    midNonProcessor_H_J_I.add(() -> ReefLocation.H);
    midNonProcessor_H_J_I.add(() -> ReefLocation.J);
    midNonProcessor_H_J_I.add(() -> ReefLocation.I);
  }

  /*
   * Paths are created using the FMS naming convention (A-L, rotating counter-clockwise from the pov of the driver stations)
   * Algae is labeled 1-6 following the same starting and ending sides as coral (1-6, rotating counter-clockwise from the pov of the driver stations)
   */

  /* Face Auto Paths */
  // Paths are labeled from the location you are coming from
  private static final PathSequence facePaths = new PathSequence("Face_EF", "Face_GH", "Algae_Off_CD", "Algae_Off_KL");

  private static final PathSequence stationPaths = new PathSequence("Station_EF", "Station_GH");

  private static final PathSequence stealPaths = new PathSequence("Steal_3", "Leave_Line");

  /* Test Auto Sequences */
  private static final PathSequence leave = new PathSequence("Leave");

  /* Push Auto Sequences */
  private static final PathSequence push_E_D_C =
      new PathSequence(
          "Processor_Push",
          "Processor-Starting_To_E",
          "E_To_Processor-Station",
          "Processor-Station_To_D",
          "D_To_Processor-Station",
          "Processor-Station_To_C");
  private static final PathSequence push_J_K_L =
      new PathSequence(
          "Non-Processor_Push",
          "Non-Processor-Starting_To_J",
          "J_To_Non-Processor-Station",
          "Non-Processor-Station_To_K",
          "K_To_Non-Processor-Station",
          "Non-Processor-Station_To_L");

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
  private static final PathSequence G_4N =
      new PathSequence("Mid-Starting_To_G", "G_To_4", "4_To_Barge1");
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
  private static final PathSequence barge_5N = new PathSequence("Barge1_To_5", "5_To_Barge1");

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

  public static ArrayList<Supplier<ReefLocation>> getScoreSequence() {
    return scoreSequence;
  }

  public static Command test(RobotContainer container) {

    return Commands.runOnce(() -> WristRollers.hasCoral = false).andThen(new Feed(container).andThen(Commands.print("test")));

    /* return new SequentialCommandGroup(
        grabAlgae(container, SuperstructureState.REEF_ALGAE_L2, intakeAlgae_TimeOut)); */
  }

  public static Command processor_E_B_C_D(RobotContainer container) {
    scoreSequence = processor_E_B_C_D;
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    feederAlliance_Pose =
        AllianceFlipUtil.get(
            FieldConstants.BLUE_PROCESSOR_FEEDER_ALIGN, FieldConstants.RED_PROCESSOR_FEEDER_ALIGN);

    return new SequentialCommandGroup(
        new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(0), 0.1),
        instantSetSuperstructure(container, SuperstructureState.FEEDING),
        new WaitUntilCommand(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(1), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        instantSetSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(2), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        instantSetSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(3), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)));
  }

  public static Command nonProcessor_J_A_L_K(RobotContainer container) {
    scoreSequence = nonProcessor_J_A_L_K;
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    feederAlliance_Pose =
        AllianceFlipUtil.get(
            FieldConstants.BLUE_NONPROCESSOR_FEEDER_ALIGN,
            FieldConstants.RED_NONPROCESSOR_FEEDER_ALIGN);

    return new SequentialCommandGroup(
        new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(0), 0.1),
        instantSetSuperstructure(container, SuperstructureState.FEEDING),
        new WaitUntilCommand(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(1), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        instantSetSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(2), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        instantSetSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(3), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)));
  }

  public static Command processor_E_C_D_2(RobotContainer container) {
    scoreSequence = processor_E_C_D_2;
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    feederAlliance_Pose =
        AllianceFlipUtil.get(
            FieldConstants.BLUE_PROCESSOR_FEEDER_ALIGN, FieldConstants.RED_PROCESSOR_FEEDER_ALIGN);

    return new SequentialCommandGroup(
        new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(0), 0.1),
        instantSetSuperstructure(container, SuperstructureState.FEEDING),
        new WaitUntilCommand(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(1), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        instantSetSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(2), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        grabAlgae(container, SuperstructureState.REEF_ALGAE_L1, intakeAlgae_TimeOut),
        facePaths.getIndex(2),
        instantSetSuperstructure(container, SuperstructureState.FEEDING)
    );
  }

  public static Command nonProcessor_J_L_K_6(RobotContainer container) {
    scoreSequence = nonProcessor_J_L_K_6;
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    feederAlliance_Pose =
        AllianceFlipUtil.get(
            FieldConstants.BLUE_PROCESSOR_FEEDER_ALIGN, FieldConstants.RED_PROCESSOR_FEEDER_ALIGN);

    return new SequentialCommandGroup(
        new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(0), 0.1),
        instantSetSuperstructure(container, SuperstructureState.FEEDING),
        new WaitUntilCommand(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(1), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        instantSetSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(2), 0.5)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        grabAlgae(container, SuperstructureState.REEF_ALGAE_L1, intakeAlgae_TimeOut),
        facePaths.getIndex(3),
        instantSetSuperstructure(container, SuperstructureState.FEEDING)
    );
  }

  public static Command mid_G_E_F(RobotContainer container) {
    scoreSequence = midProcessor_G_E_F;
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    feederAlliance_Pose =
        AllianceFlipUtil.get(
            FieldConstants.BLUE_PROCESSOR_FEEDER_ALIGN, FieldConstants.RED_PROCESSOR_FEEDER_ALIGN);

    return new SequentialCommandGroup(
        new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(0), 0.1),
        setSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance),
        facePaths.getIndex(1),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                stationPaths.getIndex(0),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(1), 0.1)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        setSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new ParallelDeadlineGroup(
            // new Feed(container),
            new SequentialCommandGroup(
                new AutoAlign2(container, feederAlliance_Pose, true, feederAlignTolerance),
                stationPaths.getIndex(0),
                new AutoScore2(container, SuperstructureState.REEF_L4, scoreSequence.get(2), 0.1)
                    .withDeadline(endAutoScore())),
            new Feed(container)),
        instantSetSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4));
  }

  public static Command mid_G_4_5(RobotContainer container, boolean steal) {
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    bargeAlliance_Pose =
        AllianceFlipUtil.get(FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN);
    Pose2d bargeAlliance_Pose_Opp =
        AllianceFlipUtil.get(
            FieldConstants.BLUE_BARGE_ALIGN_FLIPPED, FieldConstants.RED_BARGE_ALIGN_FLIPPED);

    return new SequentialCommandGroup(
        new AutoScore2(container, SuperstructureState.REEF_L4, () -> ReefLocation.G, 0.1),
        grabAlgae(container, SuperstructureState.REEF_ALGAE_L1, intakeAlgae_TimeOut),
        setSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED),
        new AutoAlign2(container, bargeAlliance_Pose, true)
            .alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    instantSetSuperstructure(container, SuperstructureState.BARGE_NET)))
            .andThen(
                wristRollers
                    .applyGoalCommand(WristRollersGoal.EJECT_ALGAE)
                    .withTimeout(algaeEject_TimeOut)),
        setSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < highElevatorHeightTolerance_L4),
        new AutoScore2(container, SuperstructureState.REEF_L2, () -> ReefLocation.I, 0.1)
            .withTimeout(1.0),
        grabAlgae(container, SuperstructureState.REEF_ALGAE_L2, intakeAlgae_TimeOut),
        new AutoAlign2(container, bargeAlliance_Pose, true)
            .alongWith(instantSetSuperstructure(container, SuperstructureState.REEF_ALGAE_L1))
            .andThen(
                new SequentialCommandGroup(
                    instantSetSuperstructure(container, SuperstructureState.BARGE_NET)),
                new WaitUntilCommand(() -> elevator.atGoal(1.0)))
            .andThen(
                wristRollers
                    .applyGoalCommand(WristRollersGoal.EJECT_ALGAE)
                    .withTimeout(algaeEject_TimeOut)),
        setSuperstructure(container, SuperstructureState.FEEDING)
            .until(() -> elevator.getMeasuredHeight().getInches() < 30.0),
        new ConditionalCommand(
            new SequentialCommandGroup(
                stealPaths.getIndex(0),
                grabAlgae(container, SuperstructureState.REEF_ALGAE_L2, 0.5),
                new AutoAlign2(container, bargeAlliance_Pose_Opp, true)
                    .alongWith(instantSetSuperstructure(container, SuperstructureState.FEEDING))
            ),
            stealPaths.getIndex(1),
            () -> steal));
  }

  /* Oklahoma Autos - DO NOT TOUCH */

  public static Command leave(RobotContainer container) {
    currentSequence = leave;

    return new SequentialCommandGroup(currentSequence.getStart());
  }

  public static Command processor_Push(RobotContainer container) {
    currentSequence = push_E_D_C;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart(),
        processor_Side_E_D_C(container));
  }

  public static Command nonProcessor_Push(RobotContainer container) {
    currentSequence = push_J_K_L;
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart(),
        nonProcessor_Side_J_K_L(container));
  }

  /* Processor Starting Side Autos */
  public static Command processor_Side_E_D_C(RobotContainer container) {
    currentSequence = E_D_C;
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart().raceWith(new WaitCommand(startPath_TimeOut)),
        new AutoScore(container, SuperstructureState.REEF_L4, () -> ScoringLocation.LEFT),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance),
        // .until((() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED)),
        currentSequence.getNext().raceWith(new Feed(container)),
        currentSequence.getNext().raceWith(new Feed(container)),
        new WaitUntilCommand(() -> WristRollers.hasCoral)
            .raceWith(wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        new AutoScore(container, SuperstructureState.REEF_L4, () -> ScoringLocation.RIGHT),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance),
        // .until((() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED)),
        currentSequence.getNext().raceWith(new Feed(container)),
        currentSequence.getNext().raceWith(new Feed(container)),
        wristRollers.applyGoalCommand(WristRollersGoal.IDLE).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(() -> WristRollers.hasCoral)
            .raceWith(wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        new AutoScore(container, SuperstructureState.REEF_L4, () -> ScoringLocation.LEFT),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance)
        // .until((() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED)),
        );
  }

  /* Middle Starting Side Autos */
  public static Command middle_Side_G_4N(RobotContainer container) {
    currentSequence = G_4N;
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    Wrist wrist = container.getSubsystems().superstructure().getWrist();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart().raceWith(new WaitCommand(startPath_TimeOut)),
        new AutoScore(container, SuperstructureState.REEF_L4, () -> ScoringLocation.LEFT),
        currentSequence.getNext().withTimeout(.05), // Instantly end the path
        new AutoAlign(container, ScoringLocation.CENTER)
            .alongWith(
                setSuperstructure(container, SuperstructureState.REEF_ALGAE_L1)
                    .until(
                        () ->
                            (elevator.getGoal()
                                    == (TalonFXSubsystemGoal) ElevatorGoal.CLEAR_ALGAE_L1
                                && wrist.getGoal() == (TalonFXSubsystemGoal) WristGoal.CLEAR_ALGAE))
                    .andThen(
                        wristRollers
                            .applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)
                            .withTimeout(intakeAlgae_TimeOut)
                            .andThen(
                                wristRollers
                                    .applyGoalCommand(WristRollersGoal.IDLE_AUTO)
                                    .withTimeout(0.25)
                                    .until(
                                        () ->
                                            wristRollers.getGoal()
                                                == (TalonFXSubsystemGoal)
                                                    WristRollersGoal.IDLE_AUTO)))),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance),
        currentSequence.getNext(),
        setSuperstructure(container, SuperstructureState.BARGE_NET).until(() -> elevator.atGoal()),
        new DriveUntilAtPose(
            AllianceFlipUtil.get(FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
            container),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_ALGAE)
            .withTimeout(algaeEject_TimeOut), // Score algae 4
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance));
  }

  public static Command middle_Side_H_4N(RobotContainer container) {
    currentSequence = H_4N;
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    Wrist wrist = container.getSubsystems().superstructure().getWrist();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart().raceWith(new WaitCommand(startPath_TimeOut)),
        new AutoScore(container, SuperstructureState.REEF_L4, () -> ScoringLocation.RIGHT),
        currentSequence.getNext().withTimeout(.05), // Instantly end the path
        new AutoAlign(container, ScoringLocation.CENTER)
            .alongWith(
                setSuperstructure(container, SuperstructureState.REEF_ALGAE_L1)
                    .until(
                        () ->
                            (elevator.getGoal()
                                    == (TalonFXSubsystemGoal) ElevatorGoal.CLEAR_ALGAE_L1
                                && wrist.getGoal() == (TalonFXSubsystemGoal) WristGoal.CLEAR_ALGAE))
                    .andThen(
                        wristRollers
                            .applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)
                            .withTimeout(intakeAlgae_TimeOut)
                            .andThen(
                                wristRollers
                                    .applyGoalCommand(WristRollersGoal.IDLE_AUTO)
                                    .withTimeout(0.25)
                                    .until(
                                        () ->
                                            wristRollers.getGoal()
                                                == (TalonFXSubsystemGoal)
                                                    WristRollersGoal.IDLE_AUTO)))),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance),
        currentSequence.getNext(),
        setSuperstructure(container, SuperstructureState.BARGE_NET).until(() -> elevator.atGoal()),
        new DriveUntilAtPose(
            AllianceFlipUtil.get(FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
            container),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_ALGAE)
            .withTimeout(algaeEject_TimeOut), // Score algae 4
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance));
  }

  public static Command middle_Side_G_4N_5N(RobotContainer container) {
    return new SequentialCommandGroup(middle_Side_5N(container, middle_Side_G_4N(container)));
  }

  public static Command middle_Side_H_4N_5N(RobotContainer container) {
    return new SequentialCommandGroup(middle_Side_5N(container, middle_Side_H_4N(container)));
  }

  public static Command middle_Side_5N(RobotContainer container, Command prevCommand) {
    currentSequence = barge_5N;
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    Wrist wrist = container.getSubsystems().superstructure().getWrist();

    return new SequentialCommandGroup(
        prevCommand,
        currentSequence.getNext().withTimeout(1.5),
        new AutoAlign(container, ScoringLocation.CENTER)
            .alongWith(
                setSuperstructure(container, SuperstructureState.REEF_ALGAE_L2)
                    .until(
                        () ->
                            (elevator.getGoal()
                                    == (TalonFXSubsystemGoal) ElevatorGoal.CLEAR_ALGAE_L2
                                && wrist.getGoal() == (TalonFXSubsystemGoal) WristGoal.CLEAR_ALGAE))
                    .andThen(
                        wristRollers
                            .applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)
                            .withTimeout(intakeAlgae_TimeOut)
                            .andThen(
                                wristRollers
                                    .applyGoalCommand(WristRollersGoal.IDLE_AUTO)
                                    .withTimeout(0.25)
                                    .until(
                                        () ->
                                            wristRollers.getGoal()
                                                == (TalonFXSubsystemGoal)
                                                    WristRollersGoal.IDLE_AUTO)))),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance),
        currentSequence.getNext(),
        setSuperstructure(container, SuperstructureState.BARGE_NET).until(() -> elevator.atGoal()),
        new DriveUntilAtPose(
            AllianceFlipUtil.get(FieldConstants.BLUE_BARGE_ALIGN, FieldConstants.RED_BARGE_ALIGN),
            container),
        wristRollers
            .applyGoalCommand(WristRollersGoal.EJECT_ALGAE)
            .withTimeout(algaeEject_TimeOut), // Score algae 4
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance));
  }

  /* Non-Processor Starting Side Autos */
  public static Command nonProcessor_Side_J_K_L(RobotContainer container) {
    currentSequence = J_K_L;
    WristRollers wristRollers = container.getSubsystems().wristRollers();
    Elevator elevator = container.getSubsystems().superstructure().getElevator();

    return new SequentialCommandGroup(
        Commands.runOnce(() -> wristRollers.applyGoal(WristRollersGoal.HOLD), wristRollers),
        currentSequence.getStart().raceWith(new WaitCommand(startPath_TimeOut)),
        new AutoScore(container, SuperstructureState.REEF_L4, () -> ScoringLocation.RIGHT),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance),
        // .until((() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED)),
        currentSequence.getNext().raceWith(new Feed(container)),
        currentSequence.getNext().raceWith(new Feed(container)),
        new WaitUntilCommand(() -> WristRollers.hasCoral)
            .raceWith(wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        new AutoScore(container, SuperstructureState.REEF_L4, () -> ScoringLocation.LEFT),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance),
        // .until((() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED)),
        currentSequence.getNext().raceWith(new Feed(container)),
        currentSequence.getNext().raceWith(new Feed(container)),
        wristRollers.applyGoalCommand(WristRollersGoal.IDLE).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(() -> WristRollers.hasCoral)
            .raceWith(wristRollers.applyGoalCommand(WristRollersGoal.IDLE)),
        new AutoScore(container, SuperstructureState.REEF_L4, () -> ScoringLocation.RIGHT),
        setSuperstructure(container, SuperstructureState.FEEDING).withTimeout(applyGoal_TimeOut),
        new WaitUntilCommand(
            () -> elevator.getMeasuredHeight().getInches() < elevatorHeightTolerance)
        // .until((() -> elevator.getGoal() == (TalonFXSubsystemGoal) ElevatorGoal.FEED)),
        );
  }

  /* Helper Commands */
  public static final Command setSuperstructure(
      RobotContainer container, SuperstructureState goalState) {
    return applyTargetStateFactory.apply(goalState, container).get();
  }

  public static final Command instantSetSuperstructure(
      RobotContainer container, SuperstructureState goalState) {
    return applyTargetStateFactory.apply(goalState, container).get().withTimeout(0.1);
  }

  public static final Command endAutoScore() {
    return new SequentialCommandGroup(
        new SequentialCommandGroup(
            new WaitCommand(2.0), new WaitUntilCommand(() -> !WristRollers.hasCoral)));
  }

  public static final Command grabAlgae(
      RobotContainer container, SuperstructureState goalState, double grabTimeout) {
    Elevator elevator = container.getSubsystems().superstructure().getElevator();
    Wrist wrist = container.getSubsystems().superstructure().getWrist();
    WristRollers wristRollers = container.getSubsystems().wristRollers();

    return new SequentialCommandGroup(
        new AutoAlign(container, ScoringLocation.CENTER)
            .alongWith(
                setSuperstructure(container, goalState)
                    .until(
                        () ->
                            (elevator.getGoal() == (TalonFXSubsystemGoal) goalState.elevatorGoal
                                && wrist.getGoal() == (TalonFXSubsystemGoal) goalState.wristGoal))
                    .andThen(
                        wristRollers
                            .applyGoalCommand(WristRollersGoal.INTAKE_ALGAE)
                            .withTimeout(grabTimeout)
                            .andThen(
                                wristRollers
                                    .applyGoalCommand(WristRollersGoal.IDLE_AUTO)
                                    .withTimeout(0.25)
                                    .until(
                                        () ->
                                            wristRollers.getGoal()
                                                == (TalonFXSubsystemGoal)
                                                    WristRollersGoal.IDLE_AUTO)))
                    .andThen(instantSetSuperstructure(container, SuperstructureState.FEEDING))));
  }
}
