// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.shuffleboardAlike;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.TunerConstants;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;

    double waitForRot = 1;
    double headingAccuracy = 7;

    public AutoContainer() {

        this.autoPaths = new HashMap<String, Command>();

        this.autoPaths.put("no auto", Commands.none());

        this.autoPaths.put("test", TunerConstants.DriveTrain.runAuto("test"));

        this.autoPaths.put("Ab1M1", TunerConstants.DriveTrain.runAuto("Ab1M1"));

        this.autoPaths.put("Mb21M1", TunerConstants.DriveTrain.runAuto("Mb21M1"));

        this.autoPaths.put("Mb23", TunerConstants.DriveTrain.runAuto("Mb23"));

        this.autoPaths.put("Mb2M1", TunerConstants.DriveTrain.runAuto("Mb2M1"));

        this.autoPaths.put("Mb2M3", TunerConstants.DriveTrain.runAuto("Mb2M3"));
        
        this.autoPaths.put("Mb32M3", TunerConstants.DriveTrain.runAuto("Mb32M3"));

        this.autoPaths.put("SM5", TunerConstants.DriveTrain.runAuto("SM5"));

        this.autoPaths.put("SM4M3", TunerConstants.DriveTrain.runAuto("SM4M3"));

        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
    }
}
