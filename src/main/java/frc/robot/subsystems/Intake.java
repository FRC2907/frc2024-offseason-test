package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Control;
import frc.robot.constants.MotorControllers;
import frc.robot.util.Util;

public class Intake implements ISubsystem{
    private double setPoint;
    private double averageCurrent;

    private Double[] currentOutputArr;
    private int head;
    public boolean hasNote;

    private CANSparkMax slowMotor;
    private CANSparkMax fastMotor;

    private Intake(CANSparkMax _slowMotor, CANSparkMax _fastMotor){ 
      this.slowMotor = _slowMotor;
      this.fastMotor = _fastMotor;

      averageCurrent = Control.intake.kAverageCurrent;
      currentOutputArr = new Double[Control.intake.kArrayLength];
      for (int i = 0; i < currentOutputArr.length; i++){
        currentOutputArr[i] = averageCurrent;
      }
      this.head = 0;
      this.hasNote = true;
    }

    private static Intake instance;

    public static Intake getInstance(){
      if (instance == null){
        instance = new Intake(MotorControllers.intakeSlow(), MotorControllers.intakeFast());
      }
      return instance;
    }



    public double getVelocity(){
      return this.fastMotor.getEncoder().getVelocity();
    }

    public void setSetPoint(double _setPoint){
      this.setPoint = _setPoint;
    }
    public boolean reachedSetPoint(){
      return Math.abs(Math.abs(this.fastMotor.getEncoder().getVelocity()) - Math.abs(setPoint))
           < Control.intake.kVelocityHysteresis;
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
    public boolean hasNote(){
      return this.hasNote; 
    } 

    public void currentDetection(){ //TODO test
      if (getCurrentSpike()){
        this.hasNote = true;
      } else {
        this.head += 1;
        this.head %= currentOutputArr.length;
        this.currentOutputArr = Util.arrayReplace(currentOutputArr, head, this.fastMotor.getOutputCurrent());
        this.averageCurrent   = Util.arrayAverage(currentOutputArr);
      }
    }


    
    @Override
    public void onLoop(){
      receiveOptions();
      this.fastMotor.getPIDController().setReference(this.setPoint, ControlType.kVelocity);
      this.slowMotor.getPIDController().setReference(this.setPoint / 2, ControlType.kVelocity);

      currentDetection();

      submitTelemetry();
    }

    @Override
    public void submitTelemetry(){
      SmartDashboard.putNumber("intake_velocity", this.getVelocity()); //consider adding velocities for both motors
      SmartDashboard.putNumber("intake_setpoint", this.setPoint);
      SmartDashboard.putBoolean("intake_hasNote", this.hasNote());
    }

    @Override
    public void receiveOptions(){}
}