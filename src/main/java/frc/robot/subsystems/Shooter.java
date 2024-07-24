package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.Util;

public class Shooter implements ISubsystem{
    private double setPoint;

    private CANSparkMax motor;
    private NetworkTable NT;
    private DoublePublisher p_velocity;

    private Shooter(CANSparkMax _motor){
        motor = _motor;
        this.motor.getEncoder().setVelocityConversionFactor(1 / frc.robot.constants.Control.shooter.ENCODER_RPM_PER_WHEEL_RPM);
        this.NT = NetworkTableInstance.getDefault().getTable("shooter");
        this.p_velocity = this.NT.getDoubleTopic("velocity").publish();
    }

    private static Shooter instance;

    public static Shooter getInstance(){
        if (instance == null){
            CANSparkMax motor = Util.createSparkGroup(frc.robot.constants.Ports.can.shooter.MOTORS, false, true);

            instance = new Shooter(motor);
        }
        return instance;
    }

    public void setSetPoint(double _setPoint){
        this.setPoint = _setPoint;
    }

    public void off(){
        this.setSetPoint(frc.robot.constants.Control.shooter.kOff);
    }

    public void amp(){
        this.setSetPoint(frc.robot.constants.Control.shooter.kAmpRPM);
    }

    public void speaker(){
        this.setSetPoint(frc.robot.constants.Control.shooter.kSpeakerRPM);
    }

    public double getVelocity(){
        return this.motor.getEncoder().getVelocity();
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