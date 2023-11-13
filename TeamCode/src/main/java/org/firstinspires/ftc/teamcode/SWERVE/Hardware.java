package org.firstinspires.ftc.teamcode.SWERVE;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hardware {
    public DcMotorEx FataSt;
    public DcMotorEx FataDr;
    public DcMotorEx SpateDr;
    public DcMotorEx SpateSt;
    public CRServo ServoFataSt;
    public CRServo ServoFataDr;
    public CRServo ServoSpateSt;
    public CRServo ServoSpateDr;
    public AnalogInput EncoderFataDr;
    public AnalogInput EncoderFataSt;
    public AnalogInput EncoderSpateDr;
    public AnalogInput EncoderSpateSt;
    private static Hardware instance = null;
    public boolean enabled;

    public static Hardware getInstance() {
        if (instance == null) {
            instance = new Hardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry)
    {
        FataDr= hardwareMap.get(DcMotorEx.class, "FataDr");
        FataSt= hardwareMap.get(DcMotorEx.class, "FataSt"); FataSt.setDirection(DcMotorSimple.Direction.REVERSE);
        SpateDr= hardwareMap.get(DcMotorEx.class, "SpateDr");
        SpateSt= hardwareMap.get(DcMotorEx.class, "SpateSt"); SpateSt.setDirection(DcMotorSimple.Direction.REVERSE);

        ServoFataDr= hardwareMap.get(CRServo.class, "ServoFataDr");
        ServoFataSt= hardwareMap.get(CRServo.class, "ServoFataSt"); ServoFataSt.setDirection(DcMotorSimple.Direction.REVERSE);
        ServoSpateDr= hardwareMap.get(CRServo.class, "ServoSpateDr");
        ServoSpateSt= hardwareMap.get(CRServo.class, "ServoSpateSt"); ServoSpateSt.setDirection(DcMotorSimple.Direction.REVERSE);

        EncoderFataDr= hardwareMap.get(AnalogInput.class, "EncoderFataDr");
        EncoderFataSt= hardwareMap.get(AnalogInput.class, "EncoderFataSt");
        EncoderSpateDr= hardwareMap.get(AnalogInput.class, "EncoderSpateDr");
        EncoderSpateSt= hardwareMap.get(AnalogInput.class, "EncoderSpateSt");

    }

    public void loop(Drivetrain drivetrain) {
        try {
            drivetrain.set();
            drivetrain.updateModules();
        } catch (Exception ignored) {
        }


    }

    public void write(Drivetrain drivetrain) {
        try {
            drivetrain.write();
        } catch (Exception ignored) {
        }
    }
}
