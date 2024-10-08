package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.constants.Control;
import frc.robot.constants.FieldElements;
import frc.robot.constants.MechanismDimensions;
import frc.robot.constants.MotorControllers;
import frc.robot.util.Util;

public class Shooter implements ISubsystem{
    private double setPoint;

    private Double[] currentOutputArr;
    private double averageCurrent;
    private int head;
    public boolean noteScored;

    private CANSparkMax motor;

    private Shooter(CANSparkMax _motor){
        this.motor = _motor;
        this.noteScored = false;
        this.head = 0;
        this.averageCurrent = Control.shooter.kAverageCurrent;
        this.currentOutputArr = new Double[Control.shooter.kArrayLength];
        for (int i = 0; i < currentOutputArr.length; i++){
            currentOutputArr[i] = averageCurrent;
          }
    }

    private static Shooter instance;

    public static Shooter getInstance(){
        if (instance == null){
            instance = new Shooter(MotorControllers.shooter());
        }
        return instance;
    }



    public void setSetPoint(double _setPoint){
        this.setPoint = _setPoint;
    }
    public boolean reachedSetPoint(){
        return Math.abs(Math.abs(this.motor.getEncoder().getVelocity()) - Math.abs(setPoint)) 
             < Control.shooter.kVelocityHysteresis;
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
        Translation2d robotPose = Drivetrain.getInstance().getPose().getTranslation();
        double airDistance = FieldElements.kSpeakerHole.getDistance(new Translation3d(
                                                                Units.metersToInches(robotPose.getX()), 
                                                                Units.metersToInches(robotPose.getY()),
                                                                MechanismDimensions.arm.kHeight)); 
        double airTime = 0.2;
        this.setSetPoint(airDistance / airTime);
    }
    public void manualShoot(){
        this.setSetPoint(Control.shooter.kMaxSpeed);
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
        receiveOptions();
        this.motor.getPIDController().setReference(this.setPoint, ControlType.kVelocity);
        submitTelemetry();
    }

    @Override
    public void submitTelemetry(){
        SmartDashboard.putNumber("shooter_velocity", this.getVelocity()); //consider adding velocities for both motors
        SmartDashboard.putNumber("shooter_setpoint", this.setPoint);
        SmartDashboard.putBoolean("shooter_noteScored", this.noteScored());
    }

    @Override
    public void receiveOptions(){}
}