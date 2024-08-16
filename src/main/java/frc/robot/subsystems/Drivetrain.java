package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Control;
import frc.robot.constants.MechanismDimensions;
import frc.robot.constants.Ports;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Util;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends MecanumDrive implements ISubsystem{
    private CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
    private double frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed;

    private DriveMode mode;
    private Field2d sb_field;
    private AHRS gyro;
    private MecanumDriveWheelPositions wheelPositions;
    private final MecanumDrivePoseEstimator poseEstimator;
    private LimelightHelpers.PoseEstimate limelightMeasurement;

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

      this.sb_field = new Field2d();
      SmartDashboard.putData(sb_field);

      this.wheelPositions = new MecanumDriveWheelPositions(
        frontLeftMotor. getEncoder().getPosition(), 
        frontRightMotor.getEncoder().getPosition(), 
        rearLeftMotor.  getEncoder().getPosition(), 
        rearRightMotor. getEncoder().getPosition());

      this.poseEstimator = new MecanumDrivePoseEstimator(
        MechanismDimensions.drivetrain.DRIVE_KINEMATICS, 
        gyro.getRotation2d(),
        this.wheelPositions, 
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.Degrees.of(5.0).in(Units.Radians)),
        VecBuilder.fill(0.5,  0.5,  Units.Degrees.of(30.0).in(Units.Radians))); //change numbers maybe
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

    public void setLocalDriveInputs(double xSpeed, double ySpeed, double zRotation){
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        xSpeed, ySpeed, zRotation);
      MecanumDriveWheelSpeeds wheelSpeeds = MechanismDimensions.drivetrain.DRIVE_KINEMATICS
        .toWheelSpeeds(chassisSpeeds);
      
      frontLeftSpeed =  wheelSpeeds.frontLeftMetersPerSecond;
      rearLeftSpeed =   wheelSpeeds.rearLeftMetersPerSecond;
      frontRightSpeed = wheelSpeeds.frontRightMetersPerSecond;
      rearRightSpeed =  wheelSpeeds.rearRightMetersPerSecond;
    }

    public void setFieldDriveInputs(double xSpeed, double ySpeed, double zRotation){ //TODO do the things
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        xSpeed, ySpeed, zRotation);
      chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, gyro.getRotation2d());
      MecanumDriveWheelSpeeds wheelSpeeds = MechanismDimensions.drivetrain.DRIVE_KINEMATICS
        .toWheelSpeeds(chassisSpeeds);
      
      frontLeftSpeed =  wheelSpeeds.frontLeftMetersPerSecond;
      rearLeftSpeed =   wheelSpeeds.rearLeftMetersPerSecond;
      frontRightSpeed = wheelSpeeds.frontRightMetersPerSecond;
      rearRightSpeed =  wheelSpeeds.rearRightMetersPerSecond;
    }

    public void sendMotorInputs(double frontLeft, double rearLeft, double frontRight, double rearRight){
      this.frontLeftMotor .getPIDController().setReference(frontLeft,  ControlType.kVelocity);
      this.rearLeftMotor  .getPIDController().setReference(rearLeft,   ControlType.kVelocity);
      this.frontRightMotor.getPIDController().setReference(frontRight, ControlType.kVelocity);
      this.rearRightMotor .getPIDController().setReference(rearRight,  ControlType.kVelocity);
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



    public Pose2d getPose(){
      return this.poseEstimator.getEstimatedPosition();
    }

    private void updatePoseFromSensors(){
      this.wheelPositions = new MecanumDriveWheelPositions(
        frontLeftMotor. getEncoder().getPosition(), 
        frontRightMotor.getEncoder().getPosition(), 
        rearLeftMotor.  getEncoder().getPosition(), 
        rearRightMotor. getEncoder().getPosition());

      this.poseEstimator.update(this.gyro.getRotation2d(), this.wheelPositions);

      limelightDoodad();
    }

    private void limelightDoodad(){
      if (Util.isBlue()){
        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      } else {
        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
      }
      if(limelightMeasurement.tagCount >= 1){
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        poseEstimator.addVisionMeasurement(
            limelightMeasurement.pose,
            limelightMeasurement.timestampSeconds);
      }
    }


    
    @Override
    public void onLoop(){
      updatePoseFromSensors();
      sendMotorInputs(frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed);
    }

    @Override
    public void submitTelemetry(){
      SmartDashboard.putNumber("drivetrain.fLeftVelocity",  frontLeftMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("drivetrain.rLeftVelocity",  rearLeftMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("drivetrain.fRightVelocity", frontRightMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("drivetrain.rRightVelocity", rearRightMotor.getEncoder().getVelocity());

      sb_field.setRobotPose(getPose());
    }

    @Override
    public void receiveOptions(){}
  }