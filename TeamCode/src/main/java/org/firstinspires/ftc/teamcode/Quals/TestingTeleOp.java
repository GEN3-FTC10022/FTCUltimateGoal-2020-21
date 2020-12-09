package org.firstinspires.ftc.teamcode.Quals;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp: Testing")
public class TestingTeleOp extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            // FIELD-CENTRIC DRIVE -----------------------------------------------------------------

            // Display rotation data
            telemetry.addData("FC Rotation (Radians): ", getHeading(true));
            telemetry.addData("FC Rotation (Degrees): ", Math.toDegrees(getHeading(true)));
            telemetry.addData("Normal Rotation (Radians): ", Math.toRadians(getHeading(false)));
            telemetry.addData("Normal Rotation (Degrees): ", getHeading(false));

        // DRIVETRAIN --------------------------------------------------------------------------

            double lefty = -gamepad1.left_stick_y;
            double leftx = gamepad1.left_stick_x;
            double rightx = gamepad1.right_stick_x;

            // Joystick deadzones prevents unintentional drivetrain movements
            if (Math.abs(lefty) <= 0.2)
                lefty = 0;

            if (Math.abs(leftx) <= 0.2)
                leftx = 0;

            if (Math.abs(rightx) <= 0.2)
                rightx = 0;

            // Motor powers are set to the power of 3 so that the drivetrain motors accelerates
            // exponentially instead of linearly
            flpower = Math.pow((lefty + leftx + rightx), 3);
            blpower = Math.pow((lefty - leftx + rightx), 3);
            frpower = Math.pow((lefty -leftx - rightx), 3);
            brpower = Math.pow((lefty + leftx - rightx), 3);

            // Motor Power is halved while the right trigger is held down to allow for more
            // precise robot control
            if (gamepad1.right_trigger > 0.8) {

                flpower /= 3;
                frpower /= 3;
                blpower /= 3;
                brpower /= 3;
            }

            else if (gamepad1.right_trigger > 0.1) {
                flpower *= (-0.8 * gamepad1.right_trigger + 1);
                frpower *= (-0.8 * gamepad1.right_trigger + 1);
                blpower *= (-0.8 * gamepad1.right_trigger + 1);
                brpower *= (-0.8 * gamepad1.right_trigger + 1);
            }

            // Set Motor Powers
            frontLeft.setPower(flpower);
            backLeft.setPower(blpower);
            frontRight.setPower(frpower);
            backRight.setPower(brpower);

            telemetry.update();
        }
    }
}

