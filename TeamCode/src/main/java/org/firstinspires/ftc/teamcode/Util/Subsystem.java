package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Subsystem {

    // Definitely not a pokemon reference
    public static HardwareMap hm;
    public static Telemetry tm;

    /**
     * Sets the hardware map and telemetry objects to be used across all subsystems. This method
     * should be called before any subclass is initialized and should not be called by any
     * particular subclass. The hardware map and telemetry objects should be the same objects used
     * within any OpMode.
     * @param hardwareMap The hardware map to be used for device configuration for all subsystems.
     * @param telemetry The telemetry to be used for logging for all subsystems.
     */
    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        hm = hardwareMap;
        tm = telemetry;

        tm.addLine("Hardware Map and Telemetry initialized");
        tm.update();
        sleep(500);
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
