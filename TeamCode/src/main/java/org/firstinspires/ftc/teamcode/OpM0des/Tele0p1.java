package org.firstinspires.ftc.teamcode.OpM0des;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SWERVE.Drivetrain;
import org.firstinspires.ftc.teamcode.SWERVE.Hardware;

import java.util.NavigableMap;

@TeleOp(name = "Tele0p1")
public class Tele0p1 extends CommandOpMode {
    private Hardware robot = Hardware.getInstance();
    private Drivetrain drive;
    double loopTime=0;
    //int servonr=0;
    //double[] offSetValue= new double[4];


    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.enabled = true;
        robot.init(hardwareMap, telemetry);

        drive=new Drivetrain(robot);
    }


    @Override
    public void run(){

        CommandScheduler.getInstance().run();
        robot.write(drive);
        robot.loop(drive);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        telemetry.addData("X: ",);
//        telemetry.addData("Y: ",);
//        telemetry.addData("H: ",);
        telemetry.update();
        loopTime = loop;
    }
}