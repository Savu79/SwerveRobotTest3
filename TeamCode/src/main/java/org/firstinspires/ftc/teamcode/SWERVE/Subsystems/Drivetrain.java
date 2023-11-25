package org.firstinspires.ftc.teamcode.SWERVE.Subsystems;


import androidx.core.math.MathUtils;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve;
import org.firstinspires.ftc.teamcode.SWERVE.SwerveModule;

public class Drivetrain{
    private HardwareSwerve robot;
    double R=Math.sqrt((robot.WHEEL_BASE*robot.WHEEL_BASE) + (robot.TRACKWIDTH*robot.TRACKWIDTH));
    public SwerveModule ModulFataDr, ModulFataSt, ModulSpateDr, ModulSpateSt;
    public SwerveModule[] modules;
    //public static double frontLeftOffset = -2.65, frontRightOffset = -3.66, backLeftOffset = -1.91, backRightOffset = -1.92;
    public static double frontLeftOffset = 0, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = 0;

    double[] ws = new double[4];
    double[] wa = new double[4];
    public double max;

    public Drivetrain(HardwareSwerve robot) {
        this.robot = robot;
        ModulFataDr = new SwerveModule(robot.FataDr, robot.ServoFataDr, robot.EncoderFataDr);
        ModulFataSt = new SwerveModule(robot.FataSt, robot.ServoFataSt, robot.EncoderFataSt);
        ModulSpateDr = new SwerveModule(robot.SpateDr, robot.ServoSpateDr, robot.EncoderSpateDr);
        ModulSpateSt = new SwerveModule(robot.SpateSt, robot.ServoSpateSt, robot.EncoderSpateSt);

        modules = new SwerveModule[]{ModulFataDr, ModulFataSt, ModulSpateDr, ModulSpateSt};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void calculeaza(double x, double y, double h){
        double STR=x; //Strafe
        double FWD=y; //Forward
        double RCW=h; //Rotate Clockwise
        double a = STR-(RCW*(robot.WHEEL_BASE/R));
        double b = STR+(RCW*(robot.WHEEL_BASE/R));
        double c = FWD-(RCW*(robot.TRACKWIDTH/R));
        double d = FWD+(RCW*(robot.TRACKWIDTH/R));

        this.ws= new double []{Math.sqrt(b*b+c*c), Math.sqrt(b*b+d*d), Math.sqrt(a*a+d*d), Math.sqrt(a*a+c*c)};
        this.wa= new double []{Math.atan2(b,c), Math.atan2(b,d), Math.atan2(a,d), Math.atan2(a,c)};

        //facem ca puterea maxima sa nu fie mai mare de 1, fara sa stricam proportiile si o trimitem catre module
        double max = max(this.ws);
        for(int i=0; i<4; i++)
            if(max>1)
                this.ws[i]=this.ws[i]/max;
    }
    public void write(){
        for(int i=0; i<4; i++){
            SwerveModule m= modules[i];
            m.setMotorPower(Math.abs(ws[i]));
            m.setTargetRotation(wa[i]);
        }
    }
    public void updateModules(){
        for(SwerveModule m : modules) m.update();
    }
    public static double max(double... args){
        double max = args[0];
        for(double d : args){
            if(d > max) max = d;
        }
        return max;
    }
}
