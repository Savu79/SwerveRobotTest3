package org.firstinspires.ftc.teamcode.SWERVE;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

public class SwerveModule {
    private CRServo servo;
    private AnalogInput encoder;
    private DcMotorEx motor;
    private PIDFController rotationController;
    public boolean flippus = false;
    double desiredAngle;
    double currentAngle;
    double angleError;
    double servoPower;
    public static double P = 0, I = 0, D = 0;
    public static double proportionalTerm;
    private double degPerV = 360 / 3.3;

    public SwerveModule(DcMotorEx m, CRServo s, AnalogInput e) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));

        encoder = e;
        //rotationController = new PIDFController(P, I, D, 0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update() {
        //desiredAngle = getTargetAngle();
        if (desiredAngle < 0) {
            desiredAngle = 360 + desiredAngle;
        }
        currentAngle = getCurrentAngle();

        angleError = desiredAngle - currentAngle;

        proportionalTerm = 0.007;
        servoPower = Range.clip(angleError * proportionalTerm, -1.0, 1.0);
        if (angleError > 180 || angleError < -180) {
            servoPower = -servoPower;
        }

        servo.setPower(servoPower);
    }

    private double getCurrentAngle() {
        return encoder.getVoltage() * degPerV;
    }

    private double getTargetAngle() {
        return this.desiredAngle;
    }

    public void setTargetRotation(double target) {
        this.desiredAngle = target;
    }

    public void setMotorPower(double power) {
        motor.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }
}
