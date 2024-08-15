package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.constants.Control;
import frc.robot.constants.Ports;
import frc.robot.util.Util;

public class Intake implements ISubsystem{
    private double setPoint;
    private double averageCurrent;

    private Double[] currentOutputArr;
    private int head;
    public boolean hasNote;

    private CANSparkMax motor;

    private Intake(CANSparkMax _motor){ //TODO add velocity conversion factor to other motor
      this.motor = _motor;
      _motor.setInverted(true);

      averageCurrent = Control.intake.kAverageCurrent;
      currentOutputArr = new Double[Control.intake.kArrayLength];
      for (int i = 0; i < currentOutputArr.length; i++){
        currentOutputArr[i] = averageCurrent;
      }
      this.head = 0;
      this.hasNote = false;
    }

    private static Intake instance;

    public static Intake getInstance(){
      if (instance == null){
        CANSparkMax motor = Util.createSparkGroup(Ports.CAN.intake.MOTORS, false, true);
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
    public boolean reachedSetPoint(){
      return Math.abs(this.motor.getEncoder().getVelocity()) < Control.intake.kVelocityHysteresis;
    }


    public boolean getCurrentSpike(){
      return Control.intake.ENCODER_AMPS_PER_INTAKE_MPS * Math.abs(this.averageCurrent) //consider changing this to just the last current output
                                                        - Math.abs(this.setPoint) 
           < Control.intake.kCurrentHystereis;
    }
    public void intake(){
      this.setSetPoint(Control.intake.kIntakingSpeed);
    }
    public void outake(){
      this.setSetPoint(Control.intake.kOutakingSpeed);
    }
    public void shoot(){
      this.setSetPoint(Control.intake.kShoot);
    }
    public void off(){
      this.setSetPoint(Control.intake.kOff);
    }
    public boolean isOn(){
      return Math.abs(this.motor.getEncoder().getVelocity()) > Control.intake.kOnHysteresis;
    }
    public boolean isOff(){
      return Math.abs(this.motor.getEncoder().getVelocity()) < Control.intake.kOnHysteresis;
    }
    public boolean hasNote(){
      return this.hasNote; 
    } 

    public void currentDetection(){ //TODO test
      if (getCurrentSpike()){
        this.hasNote = true;
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

      currentDetection();
    }

    @Override
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
}