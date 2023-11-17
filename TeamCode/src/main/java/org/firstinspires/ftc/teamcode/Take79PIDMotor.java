package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Take79PIDMotor extends LinearOpMode {

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
    private double P=0, I=0, D=0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        angleServo = hardwareMap.get(CRServo.class, "angle_servo");
        //enc = hardwareMap.get(Encoder.class, "enc");

        analogEncoder = hardwareMap.get(AnalogInput.class, "analog");
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        rotationController = new PIDController(P, I, D);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            rotationController.setPID(P, I , D);
            desiredAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * (180 / Math.PI);
            if (desiredAngle < 0) {
                desiredAngle = 360 + desiredAngle;
            }

            currentAngle = getCurrentAngle();
            angleError = desiredAngle - currentAngle;

            servoPower = Range.clip(rotationController.calculate(0, angleError), -1.0, 1.0);
            if (angleError>180 || angleError<-180)
            {
                servoPower=-servoPower;
            }

            angleServo.setPower(servoPower);
            motor.setPower(Math.sqrt((0 - gamepad1.left_stick_y) * (0 - gamepad1.left_stick_y) + (0 - gamepad1.left_stick_x) * (0 - gamepad1.left_stick_x)));
            telemetry.addData("Desired Angle", desiredAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Servo Power", servoPower);
            telemetry.addData("Joy X", gamepad1.left_stick_x);
            telemetry.addData("Joy Y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    private double getCurrentAngle() {
        return analogEncoder.getVoltage() * 360/3.3;
    }
}
