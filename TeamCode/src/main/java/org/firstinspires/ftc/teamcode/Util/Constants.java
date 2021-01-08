package org.firstinspires.ftc.teamcode.Util;

public class Constants {

    public Constants() { }

    // Motor Constants
    public static final double YELLOWJACKET_5202_MAX_RPM = 5400; // tested value
    public static final double NEVEREST_ORBITAL_20_TICKS_PER_REV = 537.6;
    public static final double NEVEREST_CLASSIC_40_TICKS_PER_REV = 1120;
    public static final double NEVEREST_CLASSIC_60_TICKS_PER_REV = 1680;
    public static final double YELLOWJACKET_5202_TICKS_PER_REV = 28;
    public static double[] motorTicksPerRev =
            {NEVEREST_ORBITAL_20_TICKS_PER_REV,
            NEVEREST_CLASSIC_40_TICKS_PER_REV,
            NEVEREST_CLASSIC_60_TICKS_PER_REV,
            YELLOWJACKET_5202_TICKS_PER_REV};

    // Controller Constants
    public static final int GAMEPAD_LOCKOUT = 500;
}
