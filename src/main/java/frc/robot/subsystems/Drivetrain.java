package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.constants.Ports;




public class Drivetrain extends MecanumDrive implements ISubsystem{
    private CANSparkMax fLeftMotor;
    private CANSparkMax rLeftMotor;
    private CANSparkMax fRightMotor;
    private CANSparkMax rRightMotor;

    private DriveMode mode;
    public enum DriveMode {
          AUTO

        , LOCAL_FORWARD, LOCAL_REVERSED 

        , FIELD_FORWARD, FIELD_REVERSED
    }

    private Drivetrain(CANSparkMax frontLeft, CANSparkMax rearLeft, CANSparkMax frontRight, CANSparkMax rearRight){
      super(frontLeft, rearLeft, frontRight, rearRight);
      this.fLeftMotor = frontLeft;
      this.rLeftMotor = rearLeft;
      this.fRightMotor = frontRight;
      this.rRightMotor = rearRight;
      this.mode = DriveMode.LOCAL_FORWARD; 
    }

    private static Drivetrain instance;

    public static Drivetrain getInstance(){
      CANSparkMax frontLeft, rearLeft, frontRight, rearRight;
      if (instance == null){
        frontLeft = new CANSparkMax(Ports.can.drivetrain.FRONT_LEFT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        rearLeft = new CANSparkMax(Ports.can.drivetrain.REAR_LEFT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        frontRight = new CANSparkMax(Ports.can.drivetrain.FRONT_RIGHT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        rearRight = new CANSparkMax(Ports.can.drivetrain.REAR_RIGHT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        instance = new Drivetrain(frontLeft, rearLeft, frontRight, rearRight);
      }
      return instance;
    }
    
    public void setDriveMode(DriveMode newMode){
        this.mode = newMode;
    }
  
    public DriveMode getDriveMode(){ return this.mode; }

    public void reverse(){
      if (mode == DriveMode.LOCAL_FORWARD){
        setDriveMode(DriveMode.LOCAL_REVERSED);
      } else if (mode == DriveMode.LOCAL_REVERSED){
        setDriveMode(DriveMode.LOCAL_FORWARD);
      } else if (mode == DriveMode.FIELD_FORWARD){
        setDriveMode(DriveMode.FIELD_REVERSED);
      } else if (mode == DriveMode.FIELD_REVERSED){
        setDriveMode(DriveMode.FIELD_FORWARD);
      }
    }

    public void localFieldSwitch(){
      if (mode == DriveMode.LOCAL_FORWARD){
        setDriveMode(DriveMode.FIELD_FORWARD);
      } else if (mode == DriveMode.LOCAL_REVERSED){
        setDriveMode(DriveMode.FIELD_REVERSED);
      } else if (mode == DriveMode.FIELD_FORWARD){
        setDriveMode(DriveMode.LOCAL_FORWARD);
      } else if (mode == DriveMode.FIELD_REVERSED){
        setDriveMode(DriveMode.LOCAL_REVERSED);
      }
    }
    
    private Pose2d pose;

    public Pose2d getPose(){
      return this.pose;
    }

    private void setPose(Pose2d _pose){
      this.pose = _pose;
    }

    @Override
    public void onLoop(){}

    

    @Override
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
  }