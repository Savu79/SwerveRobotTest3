package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="test extindere" , group = "test")
public class z78punct9 extends LinearOpMode {
    DcMotor motorasSmecheras;

    public void runOpMode(){

        motorasSmecheras=hardwareMap.get(DcMotor.class,"motor");
        waitForStart();
        while (opModeIsActive())
        {
            motorasSmecheras.setPower(gamepad1.left_stick_y);
            if(gamepad1.left_stick_button) motorasSmecheras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Left_stick_y: ", gamepad1.left_stick_y);
            telemetry.addData("Pozitie: ", motorasSmecheras.getCurrentPosition());
            telemetry.update();
        }
    }
}
