package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.commands.IntakeNote;
import frc.robot.auto.commands.ShootNote;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("Intake Note", new IntakeNote());
    NamedCommands.registerCommand("Shoot Note", new ShootNote());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

