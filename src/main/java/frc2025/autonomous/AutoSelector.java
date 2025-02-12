package frc2025.autonomous;

import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.RobotContainer;

public class AutoSelector {

    private enum AutoMode{
        DO_NOTHING(container -> Commands.none());

        private final Function<RobotContainer, Command> factory;

        private AutoMode(Function<RobotContainer, Command> factory){
            this.factory = factory;
        }

        public Command getCommand(RobotContainer container) {
            return factory.apply(container);
        }
    }

    private final RobotContainer container;

    private final SendableChooser<AutoMode> chooser = new SendableChooser<>();

    public AutoSelector(RobotContainer container){
        this.container = container;

        for(AutoMode mode : AutoMode.values()){
            chooser.addOption(mode.name(), mode);
        }
        chooser.setDefaultOption(AutoMode.DO_NOTHING.name(), AutoMode.DO_NOTHING);
    }
    
    public Command getAutonomousCommand(){
        return chooser.getSelected().getCommand(container);
    }
}
