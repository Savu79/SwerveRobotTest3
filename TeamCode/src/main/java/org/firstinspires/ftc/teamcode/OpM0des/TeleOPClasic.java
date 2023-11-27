package org.firstinspires.ftc.teamcode.OpM0des;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve;
import org.firstinspires.ftc.teamcode.SWERVE.Subsystems.Drivetrain;
@TeleOp(name="TeleOp Clasic")
public class TeleOPClasic extends LinearOpMode {
    HardwareSwerve robot= HardwareSwerve.getInstance();
    Drivetrain drive;
    public void runOpMode(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.enabled = true;
        robot.init(hardwareMap, telemetry);

        drive=new Drivetrain(robot);
        waitForStart();
        while (opModeIsActive()){
            robot.loop(drive, gamepad1.left_stick_x, -gamepad1.left_stick_y,gamepad1.right_stick_x);
            robot.write(drive);
        }
    }
}
