package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {

    // Definitely not a pokemon reference
    public static HardwareMap hm;
    public static Telemetry tm;

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        hm = hardwareMap;
        tm = telemetry;
    }
}
