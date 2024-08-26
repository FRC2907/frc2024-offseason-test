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

public class Arm implements ISubsystem{
    private double setPoint;
    private CANSparkMax motor;
    
    private Arm(CANSparkMax _motor){
        this.motor = _motor;
        this.setPDGains(Control.arm.kP, Control.arm.kD);
    }

    private static Arm instance;

    public static Arm getInstance(){
        if (instance == null){
            instance = new Arm(MotorControllers.arm());
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
    public void speaker(){ 
        
    /**
     * Here, we make a triangle and use trigonometry to get our angle. 
     * We get the air distance which is the hypotenuse, the flat distance which is the length,
     * and we don't need the height. The flat distance over the air distance is equal to
     * cos(angle), so we can move it over to the other side of the equation.
     * Acos is arc cosine, which is the inverse of cosine, so Acos(flatDistance / airDistance) = angle.
     */

        Translation2d robotPose = Drivetrain.getInstance().getPose().getTranslation();
        double airDistance = FieldElements.kSpeakerHole.getDistance(new Translation3d(
                                                                Units.metersToInches(robotPose.getX()), 
                                                                Units.metersToInches(robotPose.getY()),
                                                                MechanismDimensions.arm.kHeight)); 
        double flatDistance = FieldElements.kSpeakerHole.toTranslation2d().getDistance(robotPose);
        this.setSetPoint(Math.acos(flatDistance / airDistance));
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
        receiveOptions();
        this.motor.getPIDController().setReference(this.setPoint, ControlType.kPosition);
        submitTelemetry();
    }



    @Override 
    public void submitTelemetry(){
        SmartDashboard.putNumber("arm_position", this.getPosition());
        SmartDashboard.putNumber("arm_velocity", this.getVelocity());
        SmartDashboard.putNumber("arm_setpoint", this.setPoint);
    }

    @Override
    public void receiveOptions(){}
}