package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command{
    private Intake intake;

    public IntakeNote(){
        intake = Intake.getInstance();
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        intake.intake();
    }

    @Override
    public boolean isFinished(){
        if (intake.reachedSetPoint()){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        intake.off();
    }
}
