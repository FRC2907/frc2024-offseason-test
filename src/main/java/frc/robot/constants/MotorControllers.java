package frc.robot.constants;

import com.revrobotics.CANSparkMax;

import frc.robot.util.Util;

public class MotorControllers {
    private static CANSparkMax _arm, _shooter;

    public static final CANSparkMax arm(){
        if (_arm == null){
            _arm = Util.createSparkGroup(Ports.CAN.arm.MOTORS, false, true);
            _arm.getEncoder().setPositionConversionFactor(1 / Control.arm.ENCODER_POS_UNIT_PER_DEGREE);
            _arm.getEncoder().setVelocityConversionFactor(Control.arm.kVelocityConversionFactor);
            setPDGains(Control.arm.kP, Control.arm.kD, _arm);
        }
        return _arm;
    }

    public static final CANSparkMax shooter(){
        if (_shooter == null){
            _shooter = Util.createSparkGroup(Ports.CAN.shooter.MOTORS, false, true);
            _shooter.getEncoder().setVelocityConversionFactor(Control.shooter.kVelocityConversionFactor);
        }
        return _shooter;
    }



    private static CANSparkMax setPDGains(double P, double D, CANSparkMax motor){
        motor.getPIDController().setP(P);
        motor.getPIDController().setD(D);
        return motor;
    }
}
