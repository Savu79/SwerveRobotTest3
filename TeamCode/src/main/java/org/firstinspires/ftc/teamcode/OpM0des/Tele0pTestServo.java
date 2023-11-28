package org.firstinspires.ftc.teamcode.OpM0des;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve;
import org.firstinspires.ftc.teamcode.SWERVE.Subsystems.Drivetrain;

@TeleOp(name = "Tele0pTestServo")
public class Tele0pTestServo extends CommandOpMode {
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
        robot.test(drive);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        telemetry.addData("X: ",);
//        telemetry.addData("Y: ",);
//        telemetry.addData("H: ",);
        telemetry.update();
        loopTime = loop;
    }
}

