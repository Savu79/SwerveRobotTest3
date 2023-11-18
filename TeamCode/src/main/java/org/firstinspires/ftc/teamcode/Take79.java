package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "79", group = "test")
public class Take79 extends LinearOpMode {

    CRServo angleServo;
    //Encoder enc;
    AnalogInput analogEncoder;
    double desiredAngle;
    double currentAngle;
    double angleError;
    double servoPower;
    public static double proportionalTerm;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        angleServo = hardwareMap.get(CRServo.class, "angle_servo");
        //enc = hardwareMap.get(Encoder.class, "enc");

        analogEncoder = hardwareMap.get(AnalogInput.class, "analog");

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            //desiredAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
            desiredAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * (180 / Math.PI);
            if (desiredAngle < 0) {
                desiredAngle = 360 + desiredAngle;
            }
            //Math.atan2( y , x ) * ( 180 / Math.PI );
            currentAngle = getCurrentAngle();

            angleError = desiredAngle - currentAngle;

            proportionalTerm = 0.007;
            servoPower = Range.clip(angleError * proportionalTerm, -1.0, 1.0);
            if (angleError>180 || (angleError<0 && angleError>-180))
            {
                servoPower=-servoPower;
            }

            angleServo.setPower(servoPower);

            telemetry.addData("Desired Angle", desiredAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Servo Power", servoPower);
            telemetry.addData("Joy X", gamepad1.left_stick_x);
            telemetry.addData("Joy Y", gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    private double getCurrentAngle() {
        // Returneaza in grade;
        // double encoderTicks = angleServo.getCurrentPosition();
        //double enTi=enc.getCurrentPosition();
        //double degPerTi=360/1;
        double enV = analogEncoder.getVoltage();
        double degPerV = 360 / 3.3;
        // double degreesPerTick = 360.0 / encoderTicksPerRotation;
        // return encoderTicks * degreesPerTick;
        return enV * degPerV;
    }
}
