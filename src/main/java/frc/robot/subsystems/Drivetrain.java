package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.FieldElements;
import frc.robot.constants.MechanismDimensions;
import frc.robot.constants.MotorControllers;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Util;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends MecanumDrive implements ISubsystem{
    private CANSparkMax frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
    private double frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed;
    private double desiredHeading;
    private boolean rotationLock;

    private DriveMode mode;
    private Field2d sb_field;
    private AHRS gyro;
    private MecanumDriveWheelPositions wheelPositions;
    private LimelightHelpers.PoseEstimate limelightMeasurement;
    private PIDController headingController;
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
      this.rotationLock = false;
      this.desiredHeading = gyro.getAngle();
      this.headingController = new PIDController(2, 1, 1);

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
        VecBuilder.fill(0.5,  0.5,  Units.Degrees.of(30.0).in(Units.Radians))); //TODO change numbers maybe
    }

    private static Drivetrain instance;

    public static Drivetrain getInstance(){
      if (instance == null){
        instance = new Drivetrain(
                        MotorControllers.drivetrainfl(),
                        MotorControllers.drivetrainrl(), 
                        MotorControllers.drivetrainfr(),
                        MotorControllers.drivetrainrr());
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
    public void setFieldDriveInputs(double xSpeed, double ySpeed, double zRotation){ //TODO check the things? also convert from radians to degrees
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        xSpeed, ySpeed, 
        this.isLocked() ? headingController.calculate(getHeading().getDegrees()) : zRotation);
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

    public ChassisSpeeds getChassisSpeeds(double flSpeed, double frSpeed, double rlSpeed, double rrSpeed){
      MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(flSpeed, frSpeed, rlSpeed, rrSpeed);
      return MechanismDimensions.drivetrain.DRIVE_KINEMATICS.toChassisSpeeds(wheelSpeeds);
    }

    public void lock(){ //more fun trigonometry, see arm.speaker() for more details
      Translation2d speaker = FieldElements.kSpeakerHole.toTranslation2d();
      double distance = speaker.getDistance(this.getPose().getTranslation());
      double yDistance = Math.abs(speaker.getX() - this.getPose().getX());

      this.rotationLock = true;
      this.desiredHeading = Math.acos(yDistance / distance);
      this.headingController.setSetpoint(desiredHeading);
    }
    public void unlock(){
      this.rotationLock = false;
    }
    public boolean isLocked(){
      return rotationLock;
    }

    public double getXVelocity(){
      return getChassisSpeeds(frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed).vxMetersPerSecond;
    }
    public double getYVelocity(){
      return getChassisSpeeds(frontLeftSpeed, frontRightSpeed, rearLeftSpeed, rearRightSpeed).vyMetersPerSecond;
    }
    public Rotation2d getHeading(){
      return this.getPose().getRotation();
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

      //limelightDoodad();
    }

    /*private void limelightDoodad(){
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
    } */



    public void stop(){
      frontLeftSpeed = 0;
      rearLeftSpeed = 0;
      frontRightSpeed = 0;
      rearRightSpeed = 0;
    }


    
    @Override
    public void onLoop(){
      receiveOptions();
      updatePoseFromSensors();
      sendMotorInputs(frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed);
      submitTelemetry();
    }

    @Override
    public void submitTelemetry(){
      SmartDashboard.putNumber("drivetrain_fLeftVelocity",  frontLeftMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("drivetrain_rLeftVelocity",  rearLeftMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("drivetrain_fRightVelocity", frontRightMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("drivetrain_rRightVelocity", rearRightMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("drivetrain_xVelocity", getXVelocity());
      SmartDashboard.putNumber("drivetrain_yVelocity", getYVelocity());
      SmartDashboard.putNumber("drivetrain_heading", gyro.getAngle());
      SmartDashboard.putNumber("drivetrain_angularVelocity", gyro.getRate());

      sb_field.setRobotPose(getPose());
    }

    @Override
    public void receiveOptions(){}
  }