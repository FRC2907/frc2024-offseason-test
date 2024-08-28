package frc.robot.constants;

import com.revrobotics.CANSparkMax;
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
            _arm.getEncoder().setPositionConversionFactor(Control.arm.kPositionConversionFactor);
            _arm.getEncoder().setVelocityConversionFactor(Control.arm.kVelocityConversionFactor);
        }
        return _arm;
    }

    public static final CANSparkMax drivetrainfl(){
        if (_drivetrainfl == null){
            _drivetrainfl = new CANSparkMax(Ports.CAN.drivetrain.FRONT_LEFT, MotorType.kBrushless);
            _drivetrainfl.setInverted(false);
            _drivetrainfl.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        }
        return _drivetrainfl;
    }
    public static final CANSparkMax drivetrainrl(){
        if (_drivetrainrl == null){
            _drivetrainrl = new CANSparkMax(Ports.CAN.drivetrain.REAR_LEFT, MotorType.kBrushless);
            _drivetrainrl.setInverted(false);
            _drivetrainrl.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        }
        return _drivetrainrl;
    }
    public static final CANSparkMax drivetrainfr(){
        if (_drivetrainfr == null){
            _drivetrainfr = new CANSparkMax(Ports.CAN.drivetrain.FRONT_RIGHT, MotorType.kBrushless);
            _drivetrainfr.setInverted(true);
            _drivetrainfr.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        }
        return _drivetrainfr;
    }
    public static final CANSparkMax drivetrainrr(){
        if (_drivetrainrr == null){
            _drivetrainrr = new CANSparkMax(Ports.CAN.drivetrain.REAR_RIGHT, MotorType.kBrushless);
            _drivetrainrr.setInverted(true);
            _drivetrainrr.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        }
        return _drivetrainrr;
    }

    public static final CANSparkMax intakeSlow(){
        if (_intakeSlow == null){
            _intakeSlow = new CANSparkMax(Ports.CAN.intake.SLOW_MOTOR, MotorType.kBrushless);
            _intakeSlow.setInverted(true);
            _intakeSlow.getEncoder().setVelocityConversionFactor(Control.intake.kVelocityConversionFactor);
        }
        return _intakeSlow;
    }
    public static final CANSparkMax intakeFast(){
        if (_intakeFast == null){
            _intakeFast = new CANSparkMax(Ports.CAN.intake.FAST_MOTOR, MotorType.kBrushless);
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
