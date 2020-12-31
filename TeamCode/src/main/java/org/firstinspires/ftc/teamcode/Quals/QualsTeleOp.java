package org.firstinspires.ftc.teamcode.Quals;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
        Controls:
        A:
        B:
        X:
        Y:

        Up: Increase Shooter Speed
        Down: Decrease Shooter Speed
        Left:
        Right:

        Left Stick X:
        Left Stick Y:
        Left Stick Button:
        Right Stick X:
        Right Stick Y:
        Right Stick Button:

        Left Bumper:
        Left Trigger:
        Right Bumper: Single-Fire
        Right Trigger:

        Start:
        Back:
 */

@TeleOp(name = "TeleOp: Quals")
public class QualsTeleOp extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            drive();

            /*

            if (gamepad1.a && constants.a == 0) {
                constants.a = 1;
            } else if (!gamepad1.a && constants.a == 1) {
                wobbleMech.setArmPosition(2);
                constants.a = 2;
            } else if (gamepad1.a && constants.a == 2) {
                constants.a = 3;
            } else if (!gamepad1.a && constants.a == 3) {
                wobbleMech.setArmPosition(1);
                constants.a = 0;
            } else if (gamepad1.b && constants.b == 0) {
                constants.b = 1;
            } else if (!gamepad1.b && constants.b == 1) {
                wobbleMech.setArmPosition(0);
                constants.a = 0;
                constants.b = 0;
            }

             */

            if (gamepad1.x && constants.x == 0) {
                constants.x = 1;
            } else if (!gamepad1.x && constants.x == 1) {
                wobbleMech.clawOpen();
                constants.x = 2;
            } else if (gamepad1.x && constants.x == 2) {
                constants.x = 3;
            } else if (!gamepad1.x && constants.x == 3) {
                wobbleMech.clawClose();
                constants.x = 0;
            }

            telemetry.addLine("Robot Initialized");
            telemetry.addLine("WM Motor Position: " + wobbleMech.getArmPosition());
            telemetry.addLine("WM Motor RunMode: " + wobbleMech.arm.getMode());
            telemetry.addLine("WM Servo Position: " + wobbleMech.lClaw.getPosition());
            telemetry.update();
        }
    }
}

