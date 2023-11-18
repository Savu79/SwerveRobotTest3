package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Take79PID", group = "test")
public class Take79PID extends LinearOpMode {

    CRServo angleServo;
    //Encoder enc;
    AnalogInput analogEncoder;
    DcMotorEx motor;
    double desiredAngle;
    double currentAngle;
    double angleError;
    double servoPower;
    public static double proportionalTerm;
    private PIDController rotationController;
    public static double P = 0.1, I = 0.0001, D = 0.0001;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        angleServo = hardwareMap.get(CRServo.class, "angle_servo");
        //enc = hardwareMap.get(Encoder.class, "enc");

        analogEncoder = hardwareMap.get(AnalogInput.class, "analog");
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        rotationController = new PIDController(P, I, D);
//        rotationController.setIntegrationBounds();

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            rotationController.setPID(P, I, D);
            desiredAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
            if (desiredAngle < 0) {
                desiredAngle = 360 + desiredAngle;
            }

            currentAngle = getCurrentAngle();
            angleError = desiredAngle - currentAngle;

            servoPower = Range.clip(rotationController.calculate(0, angleError), -1.0, 1.0);
            if (angleError > 180 || (angleError < 0 && angleError > -180)) {
                servoPower = -servoPower;
            }

            angleServo.setPower(servoPower);
            //motor.setPower(Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));
            telemetry.addData("Desired Angle", desiredAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Servo Power", servoPower);
            telemetry.addData("Joy X", gamepad1.left_stick_x);
            telemetry.addData("Joy Y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    private double getCurrentAngle() {
        return analogEncoder.getVoltage() * 360 / 3.3;
    }
}
