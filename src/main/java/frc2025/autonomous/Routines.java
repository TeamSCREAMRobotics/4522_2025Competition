package frc2025.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2025.RobotContainer;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.superstructure.Superstructure;
import frc2025.subsystems.superstructure.SuperstructureConstants.SuperstructureState;
import frc2025.subsystems.superstructure.wrist.WristRollers;
import frc2025.subsystems.superstructure.wrist.WristRollers.WristRollersGoal;
import util.PathSequence;

public class Routines {
    private static PathSequence currentSequence;
    private static double eject_TimeOut = 0.25;
    private static double intakeAlgae_TimeOut = 0.25;

    /* 
    * Paths are created using the FMS naming convention (A-L, rotating counter-clockwise from the pov of the driver stations)
    * Algae is labeled 1-6 following the same starting and ending sides as coral (1-6, rotating counter-clockwise from the pov of the driver stations)
    */

    /* Test Auto Sequences */
    private static final PathSequence test = new PathSequence("Processor-Starting_To_E", "E_To_Processor-Station", "Processor-Station_To_D");

    /* Processor Starting Side Sequences */
    private static final PathSequence E_D_C_B_1 = new PathSequence("Processor-Starting_To_E", "E_To_Processor-Station", "Processor-Station_To_D", "D_To_Processor-Station", "Processor-Station_To_C", "C_To_Processor-Station", "Processor-Station_To_B", "B_To_1");
    private static final PathSequence E_B_C_D_2 = new PathSequence("Processor-Starting_To_E", "E_To_Processor-Station", "Processor-Station_To_B", "B_To_Processor-Station", "Processor-Station_To_C", "C_To_Processor-Station", "Processor-Station_To_D", "D_To_2");
    private static final PathSequence D_D_C_C_2P = new PathSequence("Processor-Starting_To_D", "D_To_Processor-Station", "Processor-Station_To_D", "D_To_Processor-Station", "Processor-Station_To_C", "C_To_Processor-Station", "Processor-Station_To_C", "C_To_2", "2_To_Processor");
    private static final PathSequence E_C_C_D_D_2 = new PathSequence("Processor-Starting_To_E", "E_To_Processor-Station", "Processor-Station_To_C", "C_To_Processor-Station", "Processor-Station_To_C", "C_To_Processor-Station", "Processor-Station_To_D", "D_To_Processor-Station", "Processor-Station_To_D", "D_To_2");

    
    /* Middle Starting Side Sequences */
        /* Initial Move is G */
    private static final PathSequence G_4N = new PathSequence();
    // private static final PathSequence G_4P = new PathSequence();
    private static final PathSequence G_4N_5N_3N = new PathSequence();
    // private static final PathSequence G_4N_5N_3P = new PathSequence();
    private static final PathSequence G_4N_3N_5N = new PathSequence();
    // private static final PathSequence G_4N_3P_5N = new PathSequence();
        /* Initial Move is H */
    private static final PathSequence H_4N = new PathSequence();
    // private static final PathSequence H_4P = new PathSequence();
    private static final PathSequence H_4N_5N_3N = new PathSequence();
    // private static final PathSequence H_4N_5N_3P = new PathSequence();
    private static final PathSequence H_4N_3N_5N = new PathSequence();
    // private static final PathSequence H_4N_3P_5N = new PathSequence();

    /* Non-Processor Starting Side Sequences */
    private static final PathSequence J_A_L_K_6 = new PathSequence("Non-Processor-Starting_To_J", "J_To_Non-Processor-Station", "Non-Processor-Station_To_A", "A_To_Non-Processor-Station", "Non-Processor-Station_To_L", "L_To_Non-Processor-Station", "Non-Processor-Station_To_K", "K_To_6");
    private static final PathSequence J_K_L_A_1 = new PathSequence("Non-Processor-Starting_To_J", "J_To_Non-Processor-Station", "Non-Processor-Station_To_K", "K_To_Non-Processor-Station", "Non-Processor-Station_To_L", "L_To_Non-Processor-Station", "Non-Processor-Station_To_A", "A_To_1");
    private static final PathSequence K_K_L_L_6N2 = new PathSequence("Non-Processor-Starting_To_K", "K_To_Non-Processor-Station", "Non-Processor-Station_To_K", "K_To_Non-Processor-Station", "Non-Processor-Station_To_L", "L_To_Non-Processor-Station", "Non-Processor-Station_To_L", "L_To_6", "6_To_Barge2");
    private static final PathSequence J_L_L_K_K_6 = new PathSequence("Non-Processor-Starting_To_J", "J_To_Non-Processor-Station", "Non-Processor-Station_To_L", "L_To_Non-Processor-Station", "Non-Processor-Station_To_L", "L_To_Non-Processor-Station", "Non-Processor-Station_To_K", "K_To_Non-Processor-Station", "Non-Processor-Station_To_K", "K_To_6");


    public static final PathSequence getCurrentSequence() {
        return currentSequence;
    }

    /* Testing Autos */
    public static Command testPath(RobotContainer container) {
        currentSequence = test;
        Drivetrain drivetrain = container.getSubsystems().drivetrain();

        return new SequentialCommandGroup(
            Commands.runOnce(() -> drivetrain.resetPose(currentSequence.getStartingPose())),
            currentSequence.getIndex(0),
            currentSequence.getIndex(1),
            currentSequence.getIndex(2)
        );
    }

    public static Command processor_Side_Test(RobotContainer container) {
        currentSequence = test;
        Drivetrain drivetrain = container.getSubsystems().drivetrain();
        WristRollers wristRollers = container.getSubsystems().wristRollers();
        Superstructure superstructure = container.getSubsystems().superstructure();

        return new SequentialCommandGroup(
            Commands.runOnce(() -> drivetrain.resetPose(currentSequence.getStartingPose())),
            new ParallelRaceGroup(
                currentSequence.getIndex(0),
                setElevator(superstructure, SuperstructureState.REEF_L4)
            ),
            wristRollers.applyGoalCommand(WristRollersGoal.EJECT_CORAL).withTimeout(eject_TimeOut),
            setElevator(superstructure, SuperstructureState.HOME),
            new ParallelRaceGroup(
                currentSequence.getNext()
                // new Feed_Auto(container)
            ),
            new ParallelRaceGroup(
                currentSequence.getNext(),
                setElevator(superstructure, SuperstructureState.REEF_L4)
            ),
            wristRollers.applyGoalCommand(WristRollersGoal.EJECT_CORAL).withTimeout(eject_TimeOut),
            setElevator(superstructure, SuperstructureState.HOME)
        );
    }

    /* Processor Starting Side Autos */

    
    /* Middle Starting Side Autos */
    public static Command G_4N_5N_3N(RobotContainer container, PathSequence sequence) {
        currentSequence = G_4N_5N_3N;
        Drivetrain drivetrain = container.getSubsystems().drivetrain();
        WristRollers wristRollers = container.getSubsystems().wristRollers();
        Superstructure superstructure = container.getSubsystems().superstructure();

        return new SequentialCommandGroup(
            Commands.runOnce(() -> drivetrain.resetPose(currentSequence.getStartingPose())),
            new ParallelRaceGroup(
                currentSequence.getStart(),
                setElevator(superstructure, SuperstructureState.REEF_L4)
            ),
            wristRollers.applyGoalCommand(WristRollersGoal.EJECT_CORAL).withTimeout(eject_TimeOut), // Score Coral
            setElevator(superstructure, SuperstructureState.HOME),
            _4N_5N_3N(container, sequence)
        );
    }

    public static Command H_4N_5N_3N(RobotContainer container, PathSequence sequence) {
        currentSequence = H_4N_5N_3N;
        Drivetrain drivetrain = container.getSubsystems().drivetrain();
        WristRollers wristRollers = container.getSubsystems().wristRollers();
        Superstructure superstructure = container.getSubsystems().superstructure();

        return new SequentialCommandGroup(
            Commands.runOnce(() -> drivetrain.resetPose(currentSequence.getStartingPose())),
            new ParallelRaceGroup(
                currentSequence.getStart(),
                setElevator(superstructure, SuperstructureState.REEF_L4)
            ),
            wristRollers.applyGoalCommand(WristRollersGoal.EJECT_CORAL).withTimeout(eject_TimeOut), // Score Coral
            setElevator(superstructure, SuperstructureState.HOME),
            _4N_5N_3N(container, sequence)
        );
    }

    /* Commmonly used sequences after the initial choice of either G or H for the first coral */
    public static Command _4N_5N_3N(RobotContainer container, PathSequence sequence) {
        currentSequence = sequence;
        WristRollers wristRollers = container.getSubsystems().wristRollers();
        Superstructure superstructure = container.getSubsystems().superstructure();

        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                currentSequence.getNext(),
                setElevator(superstructure, SuperstructureState.REEF_ALGAE_L1)
            ),
            wristRollers.applyGoalCommand(WristRollersGoal.INTAKE).withTimeout(intakeAlgae_TimeOut), // Algae 4 Grabbed
            setElevator(superstructure, SuperstructureState.HOME),
            new ParallelRaceGroup(
                currentSequence.getNext(),
                setElevator(superstructure, SuperstructureState.BARGE_NET)
            ),
            wristRollers.applyGoalCommand(WristRollersGoal.EJECT_CORAL).withTimeout(eject_TimeOut), // Algae 4 Scored
            setElevator(superstructure, SuperstructureState.HOME),
            new ParallelRaceGroup(
                currentSequence.getNext(),
                setElevator(superstructure, SuperstructureState.REEF_ALGAE_L2)
            ),
            wristRollers.applyGoalCommand(WristRollersGoal.INTAKE).withTimeout(intakeAlgae_TimeOut), // Algae 5 Grabbed
            setElevator(superstructure, SuperstructureState.HOME),
            new ParallelRaceGroup(
                currentSequence.getNext(),
                setElevator(superstructure, SuperstructureState.BARGE_NET)
            ).andThen(
                wristRollers.applyGoalCommand(WristRollersGoal.EJECT_ALGAE).withTimeout(eject_TimeOut) // Algae 5 Scored
            ),
            setElevator(superstructure, SuperstructureState.HOME),
            new ParallelRaceGroup(
                currentSequence.getNext(),
                setElevator(superstructure, SuperstructureState.REEF_ALGAE_L2)
            ),
            wristRollers.applyGoalCommand(WristRollersGoal.INTAKE).withTimeout(intakeAlgae_TimeOut), // Algae 3 Grabbed
            setElevator(superstructure, SuperstructureState.HOME),
            new ParallelRaceGroup(
                currentSequence.getNext(),
                setElevator(superstructure, SuperstructureState.BARGE_NET)
            ).andThen(
                wristRollers.applyGoalCommand(WristRollersGoal.EJECT_ALGAE).withTimeout(eject_TimeOut) // Algae 3 Scored
            )
        );
    }
    
    /* Non-Processor Starting Side Autos */


    /* Helper Commands */
    public static final Command setElevator(Superstructure superstructure, SuperstructureState goalState) {
        return new InstantCommand(() -> superstructure.applyTargetState(goalState));
    }
}
