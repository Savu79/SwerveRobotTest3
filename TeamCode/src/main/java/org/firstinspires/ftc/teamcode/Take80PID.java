package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SWERVE.AbsoluteAnalogEncoder;

@Config
@TeleOp(name = "Take80PID", group = "test")
public class Take80PID extends LinearOpMode {

    CRServo angleServo;
    //Encoder enc;
    AbsoluteAnalogEncoder analogEncoder;
    AnalogInput analoginput;
    DcMotorEx motor;
    double desiredAngle;
    double currentAngle;
    double angleError;
    double servoPower;
    boolean inverted=false;
    public static double proportionalTerm;
    private PIDFController rotationController;
    public static double P = 0.6, I = 0, D = 0.1;
    public static double K_STATIC = 0.03;
    public static double MAX_SERVO = 1, MAX_MOTOR = 1;
    public static boolean MOTOR_FLIPPING = true;
    public boolean wheelFlipped;
    public boolean newTarget=false;
    public static double target = 0.0;
    public  double lastTarget = 0.0;

    private double position = 0.0;
    public double lastMotorPower = 0;
    //public static double WHEEL_RADIUS = 1.4; // in
    //public static double GEAR_RATIO = 1 / (3.5 * 1.5 * 2); // output (wheel) speed / input (motor) speed
    //public static final double TICKS_PER_REV = 28;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        angleServo = hardwareMap.get(CRServo.class, "angle_servo");
        //enc = hardwareMap.get(Encoder.class, "enc");

        analoginput = hardwareMap.get(AnalogInput.class, "analog");
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        rotationController = new PIDFController(P, I, D,0);
//        rotationController.setIntegrationBounds();


        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        analogEncoder= new AbsoluteAnalogEncoder(analoginput);
        waitForStart();

        while (opModeIsActive()) {
            /*if(gamepad1.dpad_up) inverted=true;
            if(gamepad1.dpad_down) inverted=false;
            rotationController.setPIDF(P, I, D,0);
            if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0) desiredAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
            if(gamepad1.a) desiredAngle=90;
            if(gamepad1.b) desiredAngle=0;
            if (desiredAngle < 0) {
                desiredAngle = 360 + desiredAngle;
            }

            currentAngle =analogEncoder.getCurrentPosition();
            angleError = desiredAngle - currentAngle;

            servoPower = Range.clip(rotationController.calculate(0, angleError), -1.0, 1.0);
            //if (angleError > 180 || (angleError < 0 && angleError > -180)) {
            //    servoPower = -servoPower;
            //}

            angleServo.setPower(servoPower);
            //motor.setPower(Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));
            telemetry.addData("inverted: ", inverted);
            telemetry.addData("Desired Angle", desiredAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Servo Power", servoPower);
            telemetry.addData("Joy X", gamepad1.left_stick_x);
            telemetry.addData("Joy Y", gamepad1.left_stick_y);
            telemetry.update();*/
            //if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0) target = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);

            rotationController.setPIDF(P, I, D, 0);
            double current = analogEncoder.getCurrentPosition();
            target=Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)+Math.PI/2;
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
            telemetry.addData("target: ", target);
            telemetry.addData("current: ", current);
            telemetry.addData("eroare: ", error);
            telemetry.addData("wheelFlipped: ", wheelFlipped);
            telemetry.addData("Power servo: ", power);
            telemetry.update();
        }
    }
    public double getTargetRotation() {
        return normalizeRadians(target - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(position - Math.PI);
    }

    public void setMotorPower(double power) {
        if (wheelFlipped) power *= -1;
        lastMotorPower = power;
        motor.setPower(power);
    }

    public void setTargetRotation(double target) {
        this.target = normalizeRadians(target);
    }

    private double getCurrentAngle() {
        return analogEncoder.getVoltage() * 360 / 3.3;
    }
}
