package org.firstinspires.ftc.teamcode.OpM0des;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve;
import org.firstinspires.ftc.teamcode.SWERVE.Subsystems.Drivetrain;

@TeleOp(name = "Tele0p1")
public class Tele0p1 extends CommandOpMode {
    private HardwareSwerve robot = HardwareSwerve.getInstance();
    private Drivetrain drive;
    double loopTime=0;
    //int servonr=0;
    //double[] offSetValue= new double[4];


    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
        robot.enabled = true;

        drive=new Drivetrain(robot);
    }


    @Override
    public void run(){

        CommandScheduler.getInstance().run();
        robot.write(drive);
        robot.loop(drive, gamepad1.left_stick_x, -gamepad1.left_stick_y,gamepad1.right_stick_x, 0,0,0);
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        telemetry.addData("X: ",);
//        telemetry.addData("Y: ",);
//        telemetry.addData("H: ",);
        telemetry.update();
        loopTime = loop;
    }
}
