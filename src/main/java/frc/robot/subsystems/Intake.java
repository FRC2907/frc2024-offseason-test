package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake implements ISubsystem{
    //int i_intake = 13;

    public CANSparkMax motor;

    public boolean pneumaticOn;
    public boolean intakeOn;

    private Intake(CANSparkMax _motor){
      motor = _motor;
      _motor.setInverted(true);
      intakeOn = false;
    }

    private static Intake instance;

    public static Intake getInstance(){
      if (instance == null){
        CANSparkMax motor = new CANSparkMax(frc.robot.constants.Ports.can.intake.MOTOR, MotorType.kBrushless);

        instance = new Intake(motor);
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
