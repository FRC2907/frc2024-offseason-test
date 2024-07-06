package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.util.Util;

public class Shooter implements ISubsystem{
    //int i_left_shooter = 7, i_right_shooter = 8;

    public CANSparkMax motor;

    private Shooter(CANSparkMax _motor){
        motor = _motor;
    }

    private static Shooter instance;

    public static Shooter getInstance(){
        if (instance == null){
            CANSparkMax motor = Util.createSparkGroup(frc.robot.constants.Ports.can.shooter.MOTORS, false, true);

            instance = new Shooter(motor);
        }

        return instance;
    }

    @Override
    public void onLoop(){}

    @Override
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
}
