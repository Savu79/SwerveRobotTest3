package org.firstinspires.ftc.teamcode.SWERVE;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    public SwerveModule frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    public SwerveModule[] modules;
    //TODO sa bagam offset
    //TODO public static double frontLeftOffset = -0.05, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = -0.055;
    double[] ws = new double[4];
    double[] wa = new double[4];

    double wa2;
    double ws2;

    public Drivetrain(Hardware robot) {
        frontLeftModule = new SwerveModule(robot.FataSt, robot.ServoFataSt, robot.EncoderFataSt);
        backLeftModule = new SwerveModule(robot.SpateSt, robot.ServoSpateSt, robot.EncoderSpateSt);
        backRightModule = new SwerveModule(robot.SpateDr, robot.ServoSpateDr, robot.EncoderSpateDr);
        frontRightModule = new SwerveModule(robot.FataDr, robot.ServoFataDr, robot.EncoderFataDr);

        modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //TODO what is this error ??? @Override
    public void set() {
        //ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
        //wa = new double[]{atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c)};
        wa2 = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * (180 / Math.PI);
        if (wa2 < 0) {
            wa2 = 360 + wa2;
        }
        ws2 = Math.sqrt((0 - gamepad1.left_stick_y) * (0 - gamepad1.left_stick_y) + (0 - gamepad1.left_stick_x) * (0 - gamepad1.left_stick_x));
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            m.setMotorPower(ws2);
            m.setTargetRotation(wa2);
        }
    }

    public void updateModules() {
        for (SwerveModule m : modules) m.update();
    }
}
