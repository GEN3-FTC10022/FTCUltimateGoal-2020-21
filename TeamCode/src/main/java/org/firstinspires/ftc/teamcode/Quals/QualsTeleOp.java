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

            if (gamepad1.a && a == 0) {
                a = 1;
            } else if (!gamepad1.a && a == 1) {
                wobbleMech.setMotorPosition(2);
                a = 2;
            } else if (gamepad1.a && a == 2) {
                a = 3;
            } else if (!gamepad1.a && a == 3) {
                wobbleMech.setMotorPosition(1);
                a = 0;
            } else if (gamepad1.b && b == 0) {
                b = 1;
            } else if (!gamepad1.b && b == 1) {
                wobbleMech.setMotorPosition(0);
                a = 0;
                b = 0;
            }

            if (gamepad1.x && x == 0) {
                x = 1;
            } else if (!gamepad1.x && x == 1) {
                wobbleMech.open();
                x = 2;
            } else if (gamepad1.x && x == 2) {
                x = 3;
            } else if (!gamepad1.x && x == 3) {
                wobbleMech.close();
                x = 0;
            }

            telemetry.addLine("Robot Initialized");
            telemetry.addLine("WM Motor Position: " + wobbleMech.getMotorPosition());
            telemetry.addLine("WM Motor RunMode: " + wobbleMech.motor.getMode());
            telemetry.addLine("WM Servo Position: " + wobbleMech.servo.getPosition());
            telemetry.update();
        }
    }
}

