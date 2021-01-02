package org.firstinspires.ftc.teamcode.Quals;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
        Controls:
        A: Activate Wobble Mech && Pre-load Confirm
        B: Restart Wobble Mech && Pre-load Cancel
        X:
        Y: Zero Wobble Mech

        Up: Increase Shooter Speed
        Down: Decrease Shooter Speed
        Left:
        Right:

        Left Stick X: Left/Right Strafe
        Left Stick Y: Forward/Reverse
        Left Stick Button:
        Right Stick X: Rotate
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

        telemetry.setAutoClear(true);

        waitForStart();

        while (opModeIsActive()) {

            // Telemetry
            telemetry.addData("FC Heading (Deg)", Math.toDegrees(drivetrain.getHeading(true)));
            telemetry.addData("Heading (Deg)", drivetrain.getHeading(false));
            telemetry.addLine();

            /*
            telemetry.addData("L-Encoder", drivetrain.getLeftTicks());
            telemetry.addData("R-Encoder", drivetrain.getRightTicks());
            telemetry.addData("H-Encoder", drivetrain.getHorzTicks());
            telemetry.addLine();

            telemetry.addData("X Pos", drivetrain.x);
            telemetry.addData("Y Pos", drivetrain.y);
            telemetry.addData("OdoAngle", drivetrain.odoAngle);
            telemetry.addLine();

            telemetry.addLine("Wobble Mech:");
            telemetry.addData("Arm Position", wobbleMech.armPosition);
            telemetry.addData("Arm RunMode", wobbleMech.arm.getMode());
            telemetry.addData("Claw Position", wobbleMech.clawPosition);
             */

            telemetry.update();

            drive(true);
        }
    }
}

