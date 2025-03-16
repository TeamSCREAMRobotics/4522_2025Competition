// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.FollowPathCommand;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc2025.autonomous.AutoSelector.AutoMode;
import frc2025.commands.DriveToPose;
import frc2025.logging.Logger;
import frc2025.subsystems.vision.VisionManager;
import java.util.ArrayList;
import java.util.List;
import util.RunnableUtil.RunOnce;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private List<Command> allCommands = new ArrayList<>();

  private RunOnce autoCommandReloader = new RunOnce();

  private boolean hasScheduledAutoInit = false;

  private final RobotContainer robotContainer;

  public Robot() {
    super(0.03);
    robotContainer = new RobotContainer();

    Logger.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withLogExtras(true)
            .withNtPublish(true)
            .withLogEntryQueueCapacity(2000));
    Logger.setEnabled(true);

    CommandScheduler.getInstance().onCommandInitialize((command) -> allCommands.add(command));
    CommandScheduler.getInstance().onCommandFinish((command) -> allCommands.remove(command));
    CommandScheduler.getInstance().onCommandInterrupt((command) -> allCommands.remove(command));

    FollowPathCommand.warmupCommand().schedule();
    DriveToPose.warmup(robotContainer.getSubsystems());

    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.logTelemetry();
    Dashboard.periodic();
    RobotContainer.getVisionManager().periodic();

    Logger.log(
        "AllCommands",
        allCommands.stream().map((command) -> command.getName()).toArray(String[]::new));

    autoCommandReloader.runOnceWhenChanged(
        () -> {
          autonomousCommand = robotContainer.getAutonomousCommand();
        },
        robotContainer.getAutoSelector().getSelected());
  }

  @Override
  public void disabledInit() {
    if (!hasScheduledAutoInit) {
      Command initAutoCommand =
          AutoMode.DO_NOTHING.getCommand(robotContainer).ignoringDisable(true);
      initAutoCommand.schedule();
      hasScheduledAutoInit = true;
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    VisionManager.hasEnabled = true;
    //robotContainer.getSubsystems().climber().setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void autonomousInit() {
    if (autonomousCommand != null) {
      // autonomousCommand = autonomousCommand.beforeStarting(Commands.waitSeconds(0.01));
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
  }

  @Override
  public void simulationPeriodic() {
    Dashboard.Sim.periodic();
  }
}
