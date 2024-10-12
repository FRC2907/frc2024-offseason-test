package frc.robot.constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.Util;

public class MotorControllers {
    private static CANSparkMax _arm,
                               _drivetrainfl, _drivetrainrl, _drivetrainfr, _drivetrainrr,
                               _intakeSlow, _intakeFast,
                               _shooter;

    public static final CANSparkMax arm(){
        if (_arm == null){
            _arm = Util.createSparkGroup(Ports.CAN.arm.MOTORS, false, true);
            _arm.restoreFactoryDefaults();
            _arm.getEncoder().setPositionConversionFactor(Control.arm.kPositionConversionFactor);
            _arm.getEncoder().setVelocityConversionFactor(Control.arm.kVelocityConversionFactor);
        }
        return _arm;
    }

    public static final CANSparkMax drivetrainfl(){
        if (_drivetrainfl == null){
            _drivetrainfl = new CANSparkMax(Ports.CAN.drivetrain.FRONT_LEFT, MotorType.kBrushless);
            _drivetrainfl.restoreFactoryDefaults();
            _drivetrainfl.setIdleMode(IdleMode.kBrake);
            _drivetrainfl.setInverted(false);
            _drivetrainfl.getPIDController().setFF(0.00099);
            //_drivetrainfl.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        }
        return _drivetrainfl;
    }
    public static final CANSparkMax drivetrainrl(){
        if (_drivetrainrl == null){
            _drivetrainrl = new CANSparkMax(Ports.CAN.drivetrain.REAR_LEFT, MotorType.kBrushless);
            _drivetrainrl.restoreFactoryDefaults();
            _drivetrainrl.setIdleMode(IdleMode.kBrake);
            _drivetrainrl.setInverted(false);
            _drivetrainrl.getPIDController().setFF(0.00109);
            //_drivetrainrl.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        }
        return _drivetrainrl;
    }
    public static final CANSparkMax drivetrainfr(){
        if (_drivetrainfr == null){
            _drivetrainfr = new CANSparkMax(Ports.CAN.drivetrain.FRONT_RIGHT, MotorType.kBrushless);
            _drivetrainfr.restoreFactoryDefaults();
            _drivetrainfr.setIdleMode(IdleMode.kBrake);
            _drivetrainfr.setInverted(true);
            _drivetrainfr.getPIDController().setFF(0.00107);
            //_drivetrainfr.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        }
        return _drivetrainfr;
    }
    public static final CANSparkMax drivetrainrr(){
        if (_drivetrainrr == null){
            _drivetrainrr = new CANSparkMax(Ports.CAN.drivetrain.REAR_RIGHT, MotorType.kBrushless);
            _drivetrainrr.restoreFactoryDefaults();
            _drivetrainrr.setIdleMode(IdleMode.kBrake);
            _drivetrainrr.setInverted(true);
            _drivetrainrr.getPIDController().setFF(0.00115);
           // _drivetrainrr.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        }
        return _drivetrainrr;
    }

    public static final CANSparkMax intakeSlow(){
        if (_intakeSlow == null){
            _intakeSlow = new CANSparkMax(Ports.CAN.intake.SLOW_MOTOR, MotorType.kBrushless);
            _intakeSlow.restoreFactoryDefaults();
            _intakeSlow.setInverted(true);
            _intakeSlow.getEncoder().setVelocityConversionFactor(Control.intake.kVelocityConversionFactor);
        }
        return _intakeSlow;
    }
    public static final CANSparkMax intakeFast(){
        if (_intakeFast == null){
            _intakeFast = new CANSparkMax(Ports.CAN.intake.FAST_MOTOR, MotorType.kBrushless);
            _intakeFast.restoreFactoryDefaults();
            _intakeFast.setInverted(false);
            _intakeFast.getEncoder().setVelocityConversionFactor(Control.intake.kVelocityConversionFactor);
        }
        return _intakeFast;
    }

    public static final CANSparkMax shooter(){
        if (_shooter == null){
            _shooter = Util.createSparkGroup(Ports.CAN.shooter.MOTORS, false, true);
            _shooter.getEncoder().setVelocityConversionFactor(Control.shooter.kVelocityConversionFactor);
        }
        return _shooter;
    }
}
