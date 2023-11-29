package org.firstinspires.ftc.teamcode.SWERVE.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SWERVE.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve;
import org.firstinspires.ftc.teamcode.SWERVE.SwerveModule;
@Config
public class Drivetrain{
    private HardwareSwerve robot;
    public static double TRACKWIDTH = 9, WHEEL_BASE = 9;

    double R=Math.sqrt((WHEEL_BASE*WHEEL_BASE) + (TRACKWIDTH*TRACKWIDTH));
    public SwerveModule ModulFataDr, ModulFataSt, ModulSpateDr, ModulSpateSt;
    public SwerveModule[] modules;
    //public static double frontLeftOffset = -2.65, frontRightOffset = -3.66, backLeftOffset = -1.91, backRightOffset = -1.92;
    public static double frontLeftOffset = 0, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = 0;

    double[] ws = new double[4];
    double[] wa = new double[4];
    public double max;

    public Drivetrain(HardwareSwerve robot) {
        this.robot = robot;

        ModulFataDr = new SwerveModule(robot.FataDr, robot.ServoFataDr, new AbsoluteAnalogEncoder(robot.EncoderFataDr,3.3).zero(frontRightOffset).setInverted(true));
        ModulFataSt = new SwerveModule(robot.FataSt, robot.ServoFataSt, new AbsoluteAnalogEncoder(robot.EncoderFataSt,3.3).zero(frontLeftOffset).setInverted(true));
        ModulSpateDr = new SwerveModule(robot.SpateDr, robot.ServoSpateDr, new AbsoluteAnalogEncoder(robot.EncoderSpateDr,3.3).zero(backRightOffset).setInverted(true));
        ModulSpateSt = new SwerveModule(robot.SpateSt, robot.ServoSpateSt, new AbsoluteAnalogEncoder(robot.EncoderSpateSt,3.3).zero(backLeftOffset).setInverted(true));

        modules = new SwerveModule[]{ModulFataDr, ModulFataSt, ModulSpateDr, ModulSpateSt};
        for(int i=0; i<4; i++){
            SwerveModule m= modules[i];
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void calculeaza(double x, double y, double h){
        double STR=x; //Strafe
        double FWD=y; //Forward
        double RCW=h; //Rotate Clockwise
        double a = STR-(RCW*(WHEEL_BASE/R));
        double b = STR+(RCW*(WHEEL_BASE/R));
        double c = FWD-(RCW*(TRACKWIDTH/R));
        double d = FWD+(RCW*(TRACKWIDTH/R));

        ws= new double []{Math.sqrt(b*b+c*c), Math.sqrt(b*b+d*d), Math.sqrt(a*a+d*d), Math.sqrt(a*a+c*c)};
        wa= new double []{Math.atan2(b,c), Math.atan2(b,d), Math.atan2(a,d), Math.atan2(a,c)};

        //facem ca puterea maxima sa nu fie mai mare de 1, fara sa stricam proportiile si o trimitem catre module
        double max = max(ws);
        for(int i=0; i<4; i++)
            if(max>1){
                ws[i]=ws[i]/max;
            }
    }
    public void write(){
        for(int i=0; i<4; i++){
            SwerveModule m= modules[i];
            m.setMotorPower(Math.abs(ws[i]));
            m.setTargetRotation(wa[i]);
        }
    }

    public void test()
    {
        for(int i=0; i<4; i++)
        {
            SwerveModule m = modules[i];
            m.setPowerServo();
        }
    }
    public void updateModules(){
        for(int i=0; i<4; i++){
            SwerveModule m= modules[i];
            m.update();
        };
    }
    public static double max(double... args){
        double max = args[0];
        for(double d : args){
            if(d > max) max = d;
        }
        return max;
    }
}
