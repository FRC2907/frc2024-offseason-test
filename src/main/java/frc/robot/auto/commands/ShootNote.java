package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command{
    private Arm arm;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    public ShootNote(){
        arm = Arm.getInstance();
        drivetrain = Drivetrain.getInstance();
        intake = Intake.getInstance();
        shooter = Shooter.getInstance();
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        arm.speaker();
        drivetrain.lock();
        shooter.speaker();
        if (arm.reachedSetPoint() && shooter.reachedSetPoint()){
            intake.shoot();
        }
    }

    @Override
    public boolean isFinished(){
        if (shooter.noteScored()){
            arm.neutralPosition();
            drivetrain.unlock();
            intake.off();
            shooter.off();
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        arm.neutralPosition();
        drivetrain.unlock();
        intake.off();
        shooter.off();
    }
}
