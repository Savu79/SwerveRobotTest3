package org.firstinspires.ftc.teamcode.SWERVE.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve.TRACKWIDTH;
import static org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve.WHEEL_BASE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SWERVE.DRIVETRAIN;
import org.firstinspires.ftc.teamcode.SWERVE.HardwareSwerve;

@Config
public class DriveTrain implements DRIVETRAIN {
    private HardwareSwerve robot;
    double R=Math.sqrt((WHEEL_BASE*WHEEL_BASE) + (TRACKWIDTH*TRACKWIDTH));
    public SwerveModule ModulFataDr, ModulFataSt, ModulSpateDr, ModulSpateSt;
    public SwerveModule[] modules;
    //public static double frontLeftOffset = -2.65, frontRightOffset = -3.66, backLeftOffset = -1.91, backRightOffset = -1.92;
    public static double frontLeftOffset = 0, frontRightOffset = 0, backLeftOffset = 0, backRightOffset = 0;

    double[] ws = new double[4];
    double[] wa = new double[4];
    public double max;

    public DriveTrain(HardwareSwerve robot) {
        this.robot = robot;
        ModulFataDr = new SwerveModule(robot.FataDr, robot.ServoFataDr, robot.EncoderFataDr);
        ModulFataSt = new SwerveModule(robot.FataSt, robot.ServoFataSt, robot.EncoderFataSt);
        ModulSpateDr = new SwerveModule(robot.SpateDr, robot.ServoSpateDr, robot.EncoderSpateDr);
        ModulSpateSt = new SwerveModule(robot.SpateSt, robot.ServoSpateSt, robot.EncoderSpateSt);

        modules = new SwerveModule[]{ModulFataDr, ModulFataSt, ModulSpateDr, ModulSpateSt};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void calculeaza(Pose2d Pose) // forward, strafe, rotating clockwise
    {
        double STR=Pose.position.x; //Strafe
        double FWD=Pose.position.y; //Forward
        double RCW=Pose.heading.real; //Rotate Clockwise
        double a = STR-(RCW*(WHEEL_BASE/R));
        double b = STR+(RCW*(WHEEL_BASE/R));
        double c = FWD-(RCW*(TRACKWIDTH/R));
        double d = FWD+(RCW*(TRACKWIDTH/R));

        ws= new double []{Math.sqrt(b*b+c*c), Math.sqrt(b*b+d*d), Math.sqrt(a*a+d*d), Math.sqrt(a*a+c*c)};
        wa= new double []{Math.atan2(b,c)*180/3.14, Math.atan2(b,d)*180/3.14, Math.atan2(a,d)*180/3.14, Math.atan2(a,c)*180/3.14};
        telemetry.addData("wa", wa);
        telemetry.update();

        //facem ca puterea maxima sa nu fie mai mare de 1, fara sa stricam proportiile si o trimitem catre module
        double max = max(this.ws);
        for(int i=0; i<4; i++)
            if(max>1)
                this.ws[i]=this.ws[i]/max;
    }
    public void read() {
        for (SwerveModule module : modules) module.read();
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            m.setMotorPower(Math.abs(this.ws[i]));
            m.setTargetRotation(this.wa[i]);
        }
    }

    public void updateAllModules() {
        for (SwerveModule m : modules) m.update();
    }

    public void testPozServo1(){
        ws= new double[]{0,0,0,0};
        wa= new double[]{0,0,0,0};
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            m.setMotorPower(Math.abs(ws[i]));
            m.setTargetRotation(wa[i]);
        }
    }
    public void testPozServo2(){
        ws= new double[]{0,0,0,0};
        wa= new double[]{3.14,3.14,3.14,3.14};
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            m.setMotorPower(Math.abs(ws[i]));
            m.setTargetRotation(wa[i]);
        }
    }
    public void testMotors(boolean on){
        if(on) ws= new double[]{1,1,1,1};
        else ws= new double[]{0,0,0,0};
        wa= new double[]{0,0,0,0};
        for (int i = 0; i < 4; i++) {
            SwerveModule m = modules[i];
            m.setMotorPower(Math.abs(ws[i]));
            m.setTargetRotation(wa[i]);
        }
    }
    public static double max(double... args){
        double max = args[0];
        for(double d : args){
            if(d > max) max = d;
        }
        return max;
    }

    public static double norm(double angle){
        return angle%(2*3.14);
    }

}
