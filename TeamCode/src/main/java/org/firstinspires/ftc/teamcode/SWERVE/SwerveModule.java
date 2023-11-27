package org.firstinspires.ftc.teamcode.SWERVE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.Take80PID.K_STATIC;
import static org.firstinspires.ftc.teamcode.Take80PID.MAX_SERVO;
import static org.firstinspires.ftc.teamcode.Take80PID.MOTOR_FLIPPING;

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
    private CRServo angleServo;
    private AnalogInput analoginput;
    private AbsoluteAnalogEncoder analogEncoder;
    private DcMotorEx motor;
    private PIDFController rotationController;
    public boolean flippus = false;
    double target;
    double lastTarget;
    boolean newTarget=false;
    boolean wheelFlipped=false;
    public static double P = 0.6, I = 0, D = 0.1;
    public static double proportionalTerm;
    private double degPerV = 360 / 3.3;

    public SwerveModule(DcMotorEx m, CRServo s, AnalogInput e) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleServo = s;
        ((CRServoImplEx) angleServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));

        analoginput = e;
        //rotationController = new PIDFController(P, I, D, 0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update() {
        rotationController.setPIDF(P, I, D, 0);
        double current = analogEncoder.getCurrentPosition();
        if(target==lastTarget) newTarget=false;
        else newTarget=true;

        double error = normalizeRadians(target - current);
        if (MOTOR_FLIPPING && (Math.abs(error) > Math.PI / 2)) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } else if(newTarget){
            wheelFlipped = false;
        }
        lastTarget=target;
        error = normalizeRadians(target - current);

        double power = Range.clip(rotationController.calculate(0, error), -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(power)) power = 0;
        angleServo.setPower(power + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power));
    }

    private double getCurrentAngle() {
        return analogEncoder.getVoltage() * degPerV;
    }

    private double getTargetRotation() {
        return target;
    }

    public void setTargetRotation(double target) {
        this.target = target;
    }

    public void setMotorPower(double power) {
        motor.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }
}
