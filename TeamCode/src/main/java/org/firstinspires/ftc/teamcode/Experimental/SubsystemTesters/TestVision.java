package org.firstinspires.ftc.teamcode.Experimental.SubsystemTesters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Vision;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.Subsystem;

@TeleOp(name = "Subsystems: Vision Test")
public class TestVision extends LinearOpMode {

    private boolean isAuto;

    @Override
    public void runOpMode() {

        initialize(true);

        telemetry.setAutoClear(true);

        waitForStart();

        if (isAuto)
            doAuto();
        else
            doTeleOp();
    }

    public void doTeleOp() {
        while (opModeIsActive()) {

        }
    }

    public void doAuto() {
        Vision.scanStack(false, false);
        sleep(30000);
    }

    public void initialize(boolean isAuto) {

        this.isAuto = isAuto;

        Constants.reset();

        telemetry.setAutoClear(false);

        telemetry.addLine("Initializing Robot...");
        telemetry.update();
        sleep(500);

        Subsystem.initialize(hardwareMap,telemetry);

        Vision.initialize();
    }
}