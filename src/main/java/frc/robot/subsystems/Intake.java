package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.Ports;
import frc.robot.util.Util;

public class Intake implements ISubsystem{
    private double setPoint;

    private CANSparkMax motor;
    private NetworkTable NT;
    private DoublePublisher p_velocity;
    /*public boolean pneumaticOn;
    public boolean intakeOn;*/

    private Intake(CANSparkMax _motor){ //TODO add velocity conversion factor to other motor
      motor = _motor;
      _motor.setInverted(true);
      this.NT = NetworkTableInstance.getDefault().getTable("intake");
      this.p_velocity = this.NT.getDoubleTopic("velocity").publish();
    }

    private static Intake instance;

    public static Intake getInstance(){
      if (instance == null){
        CANSparkMax motor = Util.createSparkGroup(Ports.can.intake.MOTORS, false, true);

        instance = new Intake(motor);
      }
      return instance;
    }

    public double getVelocity(){
      return this.motor.getEncoder().getVelocity();
    }

    public void setSetPoint(double _setPoint){
      this.setPoint = _setPoint;
    }

    public void intake(){
      this.setSetPoint(frc.robot.constants.Control.intake.kIntakingRPM);
    }

    public void off(){
      this.setSetPoint(frc.robot.constants.Control.intake.kOff);
    }

    public boolean hasNote(){
      return false; //Read sensor or use current detection
    }

    @Override
    public void onLoop(){
      this.motor.getPIDController().setReference(this.setPoint, ControlType.kVelocity);
    }

    @Override
    public void submitTelemetry(){
      p_velocity.set(getVelocity());
    }

    @Override
    public void receiveOptions(){}
}
