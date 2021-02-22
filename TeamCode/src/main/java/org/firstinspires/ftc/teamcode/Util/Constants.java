package org.firstinspires.ftc.teamcode.Util;

public abstract class Constants {

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
    // Gamepad 1
    public static int a = 0, b = 0, x = 0, y = 0, lBumper = 0, rBumper = 0, up = 0, down = 0, left = 0,
            right = 0, lStick = 0, rStick = 0, start = 0, back = 0;
    // Gamepad 2
    public static int a2 = 0, b2 = 0, x2 = 0, y2 = 0, lBumper2 = 0, rBumper2 = 0, up2 = 0, down2 = 0, left2 = 0,
            right2 = 0, lStick2 = 0, rStick2 = 0, start2 = 0, back2 = 0;

    public static void reset() {
        a = 0;
        b = 0;
        x = 0;
        y = 0;
        lBumper = 0;
        rBumper = 0;
        up = 0;
        down = 0;
        left = 0;
        right = 0;
        lStick = 0;
        rStick = 0;
        start = 0;
        back = 0;

        a2 = 0;
        b2 = 0;
        x2 = 0;
        y2 = 0;
        lBumper2 = 0;
        rBumper2 = 0;
        up2 = 0;
        down2 = 0;
        left2 = 0;
        right2 = 0;
        lStick2 = 0;
        rStick2 = 0;
        start2 = 0;
        back2 = 0;
    }
}
