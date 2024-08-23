package frc.robot.subsystems;

import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.Drivetrain.DriveMode;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.PS5Controller;



public class Superstructure implements ISubsystem{
    private Arm arm;
    private Drivetrain drivetrain;
    private Intake intake;
    private Led led;
    private Shooter shooter;
    private ISubsystem[] subsystems;
    
    PS5Controller driver = new PS5Controller(frc.robot.constants.Ports.HID.DRIVER);
    PS5Controller operator = new PS5Controller(frc.robot.constants.Ports.HID.OPERATOR);

    private RobotState state;
    private boolean automateScoring;

    private Trajectory trajectory;



    public enum RobotState {
      MOVING_TO_START
      , START, NEUTRAL

      , MOVING_TO_INTAKING, INTAKING, OUTAKING
      , MOVING_TO_HOLDING_NOTE, HOLDING_NOTE

      , MOVING_TO_AMP, READY_TO_SCORE_AMP, SCORING_AMP

      , MOVING_TO_SPEAKER, READY_TO_SCORE_SPEAKER, SCORING_SPEAKER

      , FOLLOWING_TRAJECTORY
    }

    private Superstructure(RobotState _state, boolean _automation){
        this.state = _state;
        this.automateScoring(_automation);
        this.arm = Arm.getInstance();
        this.drivetrain = Drivetrain.getInstance();
        this.intake = Intake.getInstance();
        this.led = Led.getInstance();
        this.shooter = Shooter.getInstance();
        this.subsystems = new ISubsystem[] { arm, drivetrain, intake, led, shooter };
    } 

    private Superstructure(){
      this(RobotState.START, true);
    }

    private static Superstructure instance;

    public static Superstructure getInstance(){
      if (instance == null){
        instance = new Superstructure(RobotState.NEUTRAL, true);
      }
      return instance;
    }

    public RobotState getState() { return this.state; }


    public void moveToStart(){
      this.state = RobotState.MOVING_TO_START;
    }
    public void start(){
      this.state = RobotState.START;
    }
    public void moveToIntaking(){
      this.state = RobotState.MOVING_TO_INTAKING;
    }
    public void intakeNote(){
      this.state = RobotState.INTAKING;
    }
    public void outakeNote(){
      this.state = RobotState.OUTAKING;
    }
    public void moveToAmp(){
      this.state = RobotState.MOVING_TO_AMP;
    }
    public void scoreInAmp(){
      this.state = RobotState.SCORING_AMP;
    }
    public void moveToSpeaker(){
      this.state = RobotState.MOVING_TO_SPEAKER;
    }
    public void scoreInSpeaker(){
      this.state = RobotState.SCORING_SPEAKER;
    }
    public void followingTrajectory(){
      this.state = RobotState.FOLLOWING_TRAJECTORY;
    }
    public void neutralPosition() {
        if (intake.hasNote()){
            this.state = RobotState.HOLDING_NOTE;
            operator.setRumble(RumbleType.kBothRumble, 0.3);
        } else {
            this.state = RobotState.NEUTRAL;
        }
    }

    public void cancelAction(){
      neutralPosition();
    }
    public void automateScoring(boolean _automation){
      this.automateScoring = _automation;
    }
    public boolean isScoringAutomated(){
      return this.automateScoring;
    }



    private void handleDriving(){
      switch (state) {
        case MOVING_TO_START:
        case START:
        case NEUTRAL:
        case MOVING_TO_HOLDING_NOTE:
        case HOLDING_NOTE:
        case MOVING_TO_INTAKING:
        case INTAKING:
        case OUTAKING:
        case MOVING_TO_SPEAKER:
        case READY_TO_SCORE_SPEAKER:
        case SCORING_SPEAKER:
        //in these states, we drive manually
        switch(drivetrain.getDriveMode()){
          case FIELD_FORWARD:
            if (Util.checkDriverDeadband(Util.getLeftMagnitude(driver)) || Util.checkDriverDeadband(driver.getRightX())){
              drivetrain.setFieldDriveInputs(driver.getLeftY(), - driver.getLeftX(), driver.getRightX());
            }
            break;
          case FIELD_REVERSED:
            if (Util.checkDriverDeadband(Util.getLeftMagnitude(driver)) || Util.checkDriverDeadband(driver.getRightX())){
              drivetrain.setFieldDriveInputs( - driver.getLeftY(), driver.getLeftX(), - driver.getRightX()); //check??
            }
            break;
          case LOCAL_FORWARD:
            if (Util.checkDriverDeadband(Util.getLeftMagnitude(driver)) || Util.checkDriverDeadband(driver.getRightX())){
              drivetrain.setLocalDriveInputs(driver.getLeftY(), - driver.getLeftX(), driver.getRightX());
            }
            break;
          case LOCAL_REVERSED:
            if (Util.checkDriverDeadband(Util.getLeftMagnitude(driver)) || Util.checkDriverDeadband(driver.getRightX())){
              drivetrain.setLocalDriveInputs( - driver.getLeftY(), driver.getLeftX(), - driver.getRightX()); //check??
            }
            break;
          default:
            break;
        }
        break;

      default:
        drivetrain.setDriveMode(DriveMode.AUTO);
      }
    } 

    private void handleInputs(){ 
      if (driver.getCircleButtonPressed() || operator.getCircleButtonPressed()){
        cancelAction();
      }



      if (driver.getCrossButtonPressed()){
        moveToIntaking();
      }
      if (driver.getR2ButtonPressed()){
        moveToSpeaker();
      }
      if (driver.getL2ButtonPressed()){
        moveToAmp();
      }
      if (driver.getR1ButtonPressed()){
        drivetrain.reverse();
      }
      if (driver.getL1ButtonPressed()){
        drivetrain.localFieldSwitch();
      }

      if (operator.getSquareButton()){
        shooter.speaker();
        if (shooter.reachedSetPoint()){
            intake.shoot();
        }
      }
      if (operator.getTriangleButtonPressed()){
        neutralPosition();
      }
      if (operator.getR2Button()){
        shooter.manualShoot();
      }
      if (operator.getR2ButtonReleased()){
        intake.shoot();
      }
      if (operator.getR1ButtonPressed()){
        outakeNote();
      }
      if (operator.getL1ButtonPressed()){
        intake.off();
      }
      if (operator.getL2Button()){ 
        intake.intake();
      }
    }

    @Override
    public void onLoop(){
      switch (this.state) {
        case MOVING_TO_START:
            arm.start();
            intake.off();
            shooter.off();
            if (arm.reachedSetPoint()){
                this.state = RobotState.START;
            }
            break;
        case START:
            break;

        case MOVING_TO_INTAKING:
            intake.intake();
            if (intake.reachedSetPoint()){
              this.state = RobotState.INTAKING;
            }
            break;
        case INTAKING:
            intake.intake();
            if (intake.hasNote()){ //TODO possibly add a timer here to make sure it goes all the way?
              shooter.noteScored = false;
              this.state = RobotState.HOLDING_NOTE;
            }
            break;

        case OUTAKING:
            intake.outake();
            if (!intake.hasNote()){
              this.state = RobotState.NEUTRAL;
            }
            break;
        case HOLDING_NOTE:
            arm.holdingPosition();
            intake.off();
            break;

        case MOVING_TO_AMP:
            arm.ampPosition();
            // TODO automatically drive up to the Amp
            if (arm.reachedSetPoint()){ // TODO add drivetrain reached set point
              this.state = RobotState.READY_TO_SCORE_AMP;
            }
            break;
        case READY_TO_SCORE_AMP:
            if (this.isScoringAutomated()){
              this.state = RobotState.SCORING_AMP;
            }
            break;
        case SCORING_AMP:
            shooter.amp();
            if (shooter.noteScored()){
              intake.hasNote = false;
              this.state = RobotState.NEUTRAL;
            }
            break;

        case MOVING_TO_SPEAKER:
            arm.speaker();
            shooter.speaker();
            if (arm.reachedSetPoint() && shooter.reachedSetPoint()){ 
              this.state = RobotState.READY_TO_SCORE_SPEAKER;
            }
            break;
        case READY_TO_SCORE_SPEAKER:
            if (this.isScoringAutomated()){
              this.state = RobotState.SCORING_SPEAKER;
            }
            break;
        case SCORING_SPEAKER:
            shooter.speaker();
            if (shooter.noteScored()) { 
              intake.hasNote = false;
              this.state = RobotState.NEUTRAL;
            }
            break;

        case NEUTRAL:
            arm.neutralPosition();
            intake.off();
            shooter.off();
            break;
        default:
            break;
      }


      handleDriving();
      handleInputs();

      for (ISubsystem s : this.subsystems){
        s.onLoop();
      }
    }

    @Override
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
    
}