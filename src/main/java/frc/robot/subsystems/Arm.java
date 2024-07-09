package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.Util;

public class Arm implements ISubsystem{
    private double setPoint;
    private CANSparkMax motor;
    private NetworkTable NT;
    private DoublePublisher p_position, p_velocity;
    
    private Arm(CANSparkMax _motor){
        this.motor = _motor;
        this.motor.getEncoder().setPositionConversionFactor(1 / frc.robot.constants.Control.arm.TICK_PER_DEGREE);
        //this.setPDGains(frc.robot.constants.Control.arm.kP, frc.robot.constants.Control.arm.kD);
        this.NT = NetworkTableInstance.getDefault().getTable("arm");
        this.p_position = this.NT.getDoubleTopic("position").publish();
        this.p_velocity = this.NT.getDoubleTopic("velocity").publish();
    }

    private static Arm instance;

    public static Arm getInstance(){
        if (instance == null){
            CANSparkMax motor = Util.createSparkGroup(frc.robot.constants.Ports.can.arm.MOTORS, false, true);

            instance = new Arm(motor);
        }
        return instance;
    }

    @Override
    public void onLoop(){
        this.motor.getPIDController().setReference(this.setPoint, ControlType.kPosition);
    }

    public void setSetPoint(double _setPoint){
        this.setPoint = Util.clamp(frc.robot.constants.Control.arm.kMinPosition, _setPoint, frc.robot.constants.Control.arm.kMinPosition);
    }

    public void up(){
        setSetPoint(setPoint + frc.robot.constants.Control.arm.kManualControlDiff);
    }

    public void down(){
        setSetPoint(setPoint + frc.robot.constants.Control.arm.kManualControlDiff);
    }

    public boolean reachedSetPoint(){
        return Math.abs(this.setPoint - this.motor.getEncoder().getPosition()) < frc.robot.constants.Control.arm.kPositionHysteresis
            && Math.abs(this.motor.getEncoder().getVelocity()) < frc.robot.constants.Control.arm.kVelocityHysteresis;
    }

    public void setPDGains(double P, double D){
        this.motor.getPIDController().setP(P);
        this.motor.getPIDController().setD(D);
    }

    public void start(){
        this.setSetPoint(frc.robot.constants.Control.arm.kStartPosition);
    }

    public void ampPosition(){
        this.setSetPoint(frc.robot.constants.Control.arm.kAmpPosition);
    }

    public void ampShootPosition(){
        this.setSetPoint(frc.robot.constants.Control.arm.kAmpShootPosition);
    }

    public void speakerPosition(){ //TODO calculate speaker position with odometry
        this.setSetPoint(frc.robot.constants.Control.arm.kSpeakerPosition);
    }

    public double getPosition(){
        return this.motor.getEncoder().getPosition();
    }

    public double getVelocity(){
        return this.motor.getEncoder().getVelocity();
    }



    @Override 
    public void submitTelemetry(){
        p_position.set(getPosition());
        p_velocity.set(getVelocity());
    }

    @Override
    public void receiveOptions(){}
}
