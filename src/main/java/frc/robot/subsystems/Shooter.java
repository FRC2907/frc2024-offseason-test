package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;
import frc.robot.util.Util;

public class Shooter implements ISubsystem{
    private double setPoint;

    private Double[] currentOutputArr;
    private double averageCurrent;
    private int head;
    public boolean noteScored;

    private CANSparkMax motor;
    private NetworkTable NT;
    private DoublePublisher p_velocity;

    private Shooter(CANSparkMax _motor){
        this.motor = _motor;
        this.motor.getEncoder().setVelocityConversionFactor(1 / Control.shooter.ENCODER_VEL_UNIT_PER_SHOOTER_MPS);

        this.noteScored = false;
        this.head = 0;
        this.averageCurrent = Control.shooter.kAverageCurrent;
        for (int i = 0; i < currentOutputArr.length; i++){
            currentOutputArr[i] = averageCurrent;
          }

        this.NT = NetworkTableInstance.getDefault().getTable("shooter");
        this.p_velocity = this.NT.getDoubleTopic("velocity").publish();
    }

    private static Shooter instance;

    public static Shooter getInstance(){
        if (instance == null){
            CANSparkMax motor = Util.createSparkGroup(Ports.CAN.shooter.MOTORS, false, true);

            instance = new Shooter(motor);
        }
        return instance;
    }



    public void setSetPoint(double _setPoint){
        this.setPoint = _setPoint;
    }
    public boolean reachedSetPoint(){
        return Math.abs(this.motor.getEncoder().getVelocity()) < Control.shooter.kVelocityHysteresis;
    }


    public boolean getCurrentSpike(){
        return Control.intake.ENCODER_AMPS_PER_INTAKE_MPS * Math.abs(this.averageCurrent) 
                                                          - Math.abs(this.setPoint) 
             < Control.intake.kCurrentHystereis;
    }
    public void off(){
        this.setSetPoint(Control.shooter.kOff);
    }
    public void amp(){
        this.setSetPoint(Control.shooter.kAmpSpeed);
    }
    public void speaker(){
        this.setSetPoint(Control.shooter.kSpeakerSpeed);
    }

    public double getVelocity(){
        return this.motor.getEncoder().getVelocity();
    }
    public boolean noteScored(){
        return this.noteScored;
    }

    public void currentDetection(){
        if (getCurrentSpike()){
          this.noteScored = true;
        } else {
          this.head += 1;
          this.head %= currentOutputArr.length;
          this.currentOutputArr = Util.arrayReplace(currentOutputArr, head, this.motor.getOutputCurrent());
          this.averageCurrent   = Util.arrayAverage(currentOutputArr);
        }
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