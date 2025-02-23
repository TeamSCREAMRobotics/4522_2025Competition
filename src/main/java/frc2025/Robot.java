// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc2025.logging.Logger;
import frc2025.subsystems.vision.VisionManager;
import java.util.ArrayList;
import java.util.List;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private List<Command> allCommands = new ArrayList<>();

  private final RobotContainer robotContainer;

  public Robot() {
    super(0.03);
    robotContainer = new RobotContainer();

    Logger.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withLogExtras(false)
            .withNtPublish(true)
            .withLogEntryQueueCapacity(5000));
    Logger.setEnabled(true);

    CommandScheduler.getInstance().onCommandInitialize((command) -> allCommands.add(command));
    CommandScheduler.getInstance().onCommandFinish((command) -> allCommands.remove(command));
    CommandScheduler.getInstance().onCommandInterrupt((command) -> allCommands.remove(command));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.logTelemetry();
    Dashboard.periodic();

    Logger.log(
        "AllCommands",
        allCommands.stream().map((command) -> command.getName()).toArray(String[]::new));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    VisionManager.hasEnabled = true;
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    VisionManager.hasEnabled = true;
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
