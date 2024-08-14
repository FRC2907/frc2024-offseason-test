package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.constants.Control;
import frc.robot.constants.MechanismDimensions;
import frc.robot.constants.Ports;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends MecanumDrive implements ISubsystem{
    private CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
    private double frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed;

    private DriveMode mode;
    private AHRS gyro;
    private MecanumDriveWheelPositions wheelPositions;
    private final MecanumDrivePoseEstimator poseEstimator;

    public enum DriveMode {
          AUTO

        , LOCAL_FORWARD, LOCAL_REVERSED 

        , FIELD_FORWARD, FIELD_REVERSED
    }

    private Drivetrain(CANSparkMax frontLeft, CANSparkMax rearLeft, CANSparkMax frontRight, CANSparkMax rearRight){
      super(frontLeft, rearLeft, frontRight, rearRight);
      this.frontLeftMotor = frontLeft;
      this.rearLeftMotor = rearLeft;
      this.frontRightMotor = frontRight;
      this.rearRightMotor = rearRight;
      this.mode = DriveMode.FIELD_FORWARD; 
      this.gyro = new AHRS(SPI.Port.kMXP);

      this.wheelPositions = new MecanumDriveWheelPositions(
        frontLeftMotor. getEncoder().getPosition(), 
        frontRightMotor.getEncoder().getPosition(), 
        rearLeftMotor.  getEncoder().getPosition(), 
        rearRightMotor. getEncoder().getPosition());

      this.poseEstimator = new MecanumDrivePoseEstimator(
        MechanismDimensions.drivetrain.DRIVE_KINEMATICS, 
        gyro.getRotation2d(),
        this.wheelPositions, 
        new Pose2d()); //TODO add the vector thingies? if needed?
    }

    private static Drivetrain instance;

    public static Drivetrain getInstance(){
      CANSparkMax frontLeft, rearLeft, frontRight, rearRight;
      if (instance == null){
        frontLeft  = new CANSparkMax(Ports.CAN.drivetrain.FRONT_LEFT,  com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        rearLeft   = new CANSparkMax(Ports.CAN.drivetrain.REAR_LEFT,   com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        frontRight = new CANSparkMax(Ports.CAN.drivetrain.FRONT_RIGHT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        rearRight  = new CANSparkMax(Ports.CAN.drivetrain.REAR_RIGHT,  com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

        frontLeft. getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        rearLeft.  getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        frontRight.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        rearRight. getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);

        instance = new Drivetrain(frontLeft, rearLeft, frontRight, rearRight);
      }
      return instance;
    }
    
    public void setDriveMode(DriveMode newMode){
        this.mode = newMode;
    }
  
    public DriveMode getDriveMode(){ return this.mode; }

    public void setFieldDriveInputs(double xSpeed, double ySpeed, double zRotation){
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        xSpeed, ySpeed, zRotation);
      MecanumDriveWheelSpeeds wheelSpeeds = MechanismDimensions.drivetrain.DRIVE_KINEMATICS
        .toWheelSpeeds(chassisSpeeds);
      
      frontLeftSpeed = wheelSpeeds.frontLeftMetersPerSecond;
      rearLeftSpeed = wheelSpeeds.rearLeftMetersPerSecond;
      frontRightSpeed = wheelSpeeds.frontRightMetersPerSecond;
      rearRightSpeed = wheelSpeeds.rearRightMetersPerSecond;
    }

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
    public void onLoop(){
      this.frontLeftMotor .getPIDController().setReference(frontLeftSpeed,  ControlType.kVelocity);
      this.rearLeftMotor  .getPIDController().setReference(rearLeftSpeed,   ControlType.kVelocity);
      this.frontRightMotor.getPIDController().setReference(frontRightSpeed, ControlType.kVelocity);
      this.rearRightMotor .getPIDController().setReference(rearRightSpeed,  ControlType.kVelocity);
    }

    @Override
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
  }