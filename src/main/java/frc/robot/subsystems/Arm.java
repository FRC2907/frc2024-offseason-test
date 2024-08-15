package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.constants.Control;
import frc.robot.constants.Ports;
import frc.robot.util.Util;

public class Arm implements ISubsystem{
    private double setPoint;
    private CANSparkMax motor;
    
    private Arm(CANSparkMax _motor){
        this.motor = _motor;
        this.motor.getEncoder().setPositionConversionFactor(1 / Control.arm.ENCODER_POS_UNIT_PER_DEGREE);
        this.motor.getEncoder().setVelocityConversionFactor(1 / Control.arm.ENCODER_VEL_UNIT_PER_DEGREE_PER_SECOND);
        this.setPDGains(Control.arm.kP, Control.arm.kD);
    }

    private static Arm instance;

    public static Arm getInstance(){
        if (instance == null){
            CANSparkMax motor = Util.createSparkGroup(Ports.CAN.arm.MOTORS, false, true);

            instance = new Arm(motor);
        }
        return instance;
    }


    public void setSetPoint(double _setPoint){
        this.setPoint = Util.clamp(Control.arm.kMinPosition, _setPoint, Control.arm.kMaxPosition);
    }
    public boolean reachedSetPoint(){
        return Math.abs(this.setPoint - this.motor.getEncoder().getPosition()) < Control.arm.kPositionHysteresis
            && Math.abs(this.motor.getEncoder().getVelocity())                 < Control.arm.kVelocityHysteresis;
    }



    public void up(){
        setSetPoint(setPoint + Control.arm.kManualControlDiff);
    }
    public void down(){
        setSetPoint(setPoint + Control.arm.kManualControlDiff);
    }
    public void start(){
        this.setSetPoint(Control.arm.kStartPosition);
    }
    public void ampPosition(){
        this.setSetPoint(Control.arm.kAmpPosition);
    }
    public void wingPosition(){ //TODO calculate speaker position with odometry
        this.setSetPoint(Control.arm.kWingPosition);
    }
    public void subwooferPosition(){ //TODO calculate speaker position with odometry
        this.setSetPoint(Control.arm.kSubwooferPosition);
    }
    public void holdingPosition(){
        this.setSetPoint(Control.arm.kHoldingPosition);
    }
    public void neutralPosition(){
        this.setSetPoint(Control.arm.kNeutralPosition);
    }

    public double getPosition(){
        return this.motor.getEncoder().getPosition();
    }
    public double getVelocity(){
        return this.motor.getEncoder().getVelocity();
    }

    public void setPDGains(double P, double D){
        this.motor.getPIDController().setP(P);
        this.motor.getPIDController().setD(D);
    }



    @Override
    public void onLoop(){
        this.motor.getPIDController().setReference(this.setPoint, ControlType.kPosition);
    }



    @Override 
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
}