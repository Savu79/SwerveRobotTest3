package org.firstinspires.ftc.teamcode.SWERVE;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class AbsoluteAnalogEncoder {
    public static double DEFAULT_RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private boolean inverted;

    public AbsoluteAnalogEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }
    public AbsoluteAnalogEncoder zero(double off){
        offset = off;
        return this;
    }
    public AbsoluteAnalogEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }
    public boolean getDirection() {
        return inverted;
    }

    private double pastPosition = 1;
    public double getCurrentPosition() {
        double pos = normalize((!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange) * Math.PI*2 - offset);
        //checks for crazy values when the encoder is close to zero
        if(!VALUE_REJECTION || Math.abs(normalizeDelta(pastPosition)) > 0.1 || Math.abs(normalizeDelta(pos)) < 1) pastPosition = pos;
        return pastPosition;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    double normalize(double theta) {   //unghiuri intre 0 si 2pi
        double normalized = theta % (2*Math.PI);
        normalized = (normalized + (2*Math.PI)) % (2*Math.PI);
        return normalized <= Math.PI ? normalized : normalized - (2*Math.PI);
    }
    double normalizeDelta (double angleDelta) { //unghiuri intre -pi si pi
        if(angleDelta<-Math.PI)
            angleDelta+=Math.PI;
        if(angleDelta>Math.PI)
            angleDelta-=Math.PI;
        return angleDelta;
    }
    public double getVoltage(){
        return encoder.getVoltage();
    }
}