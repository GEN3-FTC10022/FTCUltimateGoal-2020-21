package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Localization {

    private Encoder vlEnc, vrEnc, hEnc;
    private DcMotorEx vlMotor, vrMotor, hMotor;

    public Localization (String vlName, String vrName,String hName) {
        vlEnc = new Encoder(vlMotor, vlName);
        vrEnc = new Encoder(vrMotor, vrName);
        hEnc = new Encoder(hMotor, hName);
    }

    public void initializeLocalization() {
        vlEnc.initializeHardwareMap();
        vrEnc.initializeHardwareMap();
        hEnc.initializeHardwareMap();


    }

    public void getVertical() {

    }

    public void getHorizontal() {

    }

    public void resetDriveEncoders() {

        .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
