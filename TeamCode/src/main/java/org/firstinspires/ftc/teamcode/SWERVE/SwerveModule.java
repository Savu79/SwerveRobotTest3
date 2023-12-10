package org.firstinspires.ftc.teamcode.SWERVE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.Take80PID.K_STATIC;
import static org.firstinspires.ftc.teamcode.Take80PID.MAX_SERVO;
import static org.firstinspires.ftc.teamcode.Take80PID.MOTOR_FLIPPING;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

@Config
public class SwerveModule {
    private CRServo angleServo;
    private AbsoluteAnalogEncoder analogEncoder;
    private DcMotorEx motor;
    private PIDFController rotationController;

    double target;
    double lastTarget;
    boolean newTarget=false;
    boolean wheelFlipped=false;
    public static double P1 = 0, I1 = 0, D1 = 0;
    public static double P2 = 0, I2 = 0, D2 = 0;
    public static double P3 = 0, I3 = 0, D3 = 0;
    public static double P4 = 0, I4 = 0, D4 = 0;
    double[] P= {P1,P2,P3,P4};
    double[] I= {I1,I2,I3,I4};
    double[] D= {D1,D2,D3,D4};
    private double degPerV = 360 / 3.3;

    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e, int i) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleServo = s;
        ((CRServoImplEx) angleServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));//1000-2000

        analogEncoder = e;

        rotationController = new PIDFController(P[i], I[i], D[i], 0);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void update(int i) {
        double[] P= {P1,P2,P3,P4};
        double[] I= {I1,I2,I3,I4};
        double[] D= {D1,D2,D3,D4};
        rotationController.setPIDF(P[i], I[i], D[i], 0);

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

        //angleServo.setPower(power + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power));
        angleServo.setPower(power);
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

    public void setPowerServo()
    {
        angleServo.setPower(1);
    }

    public void setMotorPower(double power) {
        motor.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }
}
