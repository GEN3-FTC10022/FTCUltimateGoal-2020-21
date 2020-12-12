package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class Encoder {

    private DcMotorEx motor;
    private String hardwareMapName;
    private ElapsedTime elapsedTime;

    public Encoder(DcMotorEx motor, String hardwareMapName) {
        this.motor = motor;
        this.hardwareMapName = hardwareMapName;
    }

    public void initializeHardwareMap() {
        motor = (DcMotorEx)hardwareMap.dcMotor.get(hardwareMapName);
    }

    public void setVelocity() {

    }
}
