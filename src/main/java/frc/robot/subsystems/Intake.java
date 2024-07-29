package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.Control;
import frc.robot.constants.Ports;
import frc.robot.util.Util;

public class Intake implements ISubsystem{
    private double setPoint;
    private double averageCurrent;
    private int head;
    private Double[] currentOutputArr;

    public boolean hasNote;

    private CANSparkMax motor;
    private NetworkTable NT;
    private DoublePublisher p_velocity;

    private Intake(CANSparkMax _motor){ //TODO add velocity conversion factor to other motor
      this.motor = _motor;
      _motor.setInverted(true);

      averageCurrent = Control.intake.kAverageCurrent;
      currentOutputArr = new Double[Control.intake.kArrayLength];
      for (int i = 0; i < currentOutputArr.length; i++){
        currentOutputArr[i] = averageCurrent;
      }
      head = 0;

      hasNote = false;

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
    public boolean reachedSetPoint(){
      return Math.abs(this.motor.getEncoder().getVelocity()) < Control.intake.kVelocityHysteresis;
    }


    public boolean getCurrentSpike(){
      return Control.intake.ENCODER_AMPS_PER_INTAKE_MPS * Math.abs(this.averageCurrent) 
                                                        - Math.abs(this.setPoint) 
           < Control.intake.kCurrentHystereis;
    }
    public void intake(){
      this.setSetPoint(Control.intake.kIntakingSpeed);
    }
    public void outake(){
      this.setSetPoint(Control.intake.kOutakingSpeed);
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
      return this.hasNote; //Read sensor or use current detection
    } 

    public void currentDetection(){
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
    public void submitTelemetry(){
      p_velocity.set(getVelocity());
    }

    @Override
    public void receiveOptions(){}
}