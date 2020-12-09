package org.firstinspires.ftc.teamcode.Quals;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp: Quals")
public class QualsTeleOp extends QualsSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            // FIELD-CENTRIC DRIVE -----------------------------------------------------------------

            // Display rotation data
            telemetry.addData("FC Rotation (Radians): ", getHeading(true));
            telemetry.addData("FC Rotation (Degrees): ", getHeading(true));
            telemetry.addData("Normal Rotation: ", getHeading(false));
            telemetry.addData("Normal Rotation: ", getHeading(false));

            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double clockwise = gamepad1.right_stick_x;

            // Joystick deadzones
            if (Math.abs(forward) < 0.2)
                forward = 0;
            if (Math.abs(right) < 0.2)
                right = 0;
            if (Math.abs(clockwise) < 0.2)
                clockwise = 0;

            // math
            if (getHeading(true) <= 0) {       // If theta is measured clockwise from zero reference

                temp = forward * Math.cos(-getHeading(true)) + right * Math.sin(getHeading(true));
                right = -forward * Math.sin(-getHeading(true)) + right * Math.cos(getHeading(true));
                forward = temp;
            }

            if (getHeading(true) > 0) {    // If theta is measured counterclockwise from zero reference

                // Theta is reversed to account for IMU measurement
                temp = forward * Math.cos(getHeading(true)) - right * Math.sin(getHeading(true));
                right = forward * Math.sin(getHeading(true)) + right * Math.cos(getHeading(true));
                forward = temp;
            }

            // assign calculated values to the power variables
            flpower = forward + clockwise + right;
            frpower = forward - clockwise - right;
            blpower = forward + clockwise - right;
            brpower = forward - clockwise + right;

            // if you have the testing time, maybe remove this one day and see if it causes any
            // problems?
            // Find the maximum of the powers
            double max = Math.max(  Math.max(Math.abs(flpower), Math.abs(frpower)),
                                    Math.max(Math.abs(blpower), Math.abs(brpower))  );
            // Use this to make sure no motor powers are above 1 (the max value the motor can accept)
            if (max > 1) {

                flpower /= max;
                frpower /= max;
                blpower /= max;
                brpower /= max;
            }

            // Motor powers are set to the power of 3 so that the drivetrain motors accelerates
            // exponentially instead of linearly
            // Note: you may consider, in the future, moving this code block to before the
            // max > 1 code block to see if that is better or worse performance, but I think
            // it will be worse because it may mess up proportions
            flpower = Math.pow(flpower, 3);
            blpower = Math.pow(blpower, 3);
            frpower = Math.pow(frpower, 3);
            brpower = Math.pow(brpower, 3);

            // Motor Power is decreased while the right trigger is held down to allow for more
            // precise robot control
            if (gamepad1.right_trigger > 0.8) {

                flpower /= 3;
                frpower /= 3;
                blpower /= 3;
                brpower /= 3;
            }

            // If the trigger is held down, but not pressed all the way down, motor power will
            // slow down proportionally to how much the trigger is pressed
            else if (gamepad1.right_trigger > 0.1) {

                double driveSlow = -0.8 * gamepad1.right_trigger + 1;

                flpower *= driveSlow;
                frpower *= driveSlow;
                blpower *= driveSlow;
                brpower *= driveSlow;
            }

            /*
            // Alternate version of drive slowing
            // This version does not scale proportionally to the press, but uses a constant
            // multiplier instead
            // Programmers may choose to use this version of drive slowing instead due to a
            // driver's preference
            if (gamepad1.right_trigger > 0.5){

                flpower /= 3;
                blpower /= 3;
                frpower /= 3;
                brpower /= 3;
            }
            */

            // assign power to the motors
            frontLeft.setPower(-flpower);
            frontRight.setPower(-frpower);
            backLeft.setPower(-blpower);
            backRight.setPower(brpower);


            // WOBBLE MECH -------------------------------------------------------------------------

            // Wobble Clamp
            if (gamepad1.x && x == 0) {
                x = 1;
            } else if (!gamepad1.x && x == 1) {
                wobbleClamp.setPosition(wobbleClampClose);
                x = 2;
            } else if (gamepad1.x && x == 2) {
                x = 3;
            } else if (!gamepad1.x && x == 3) {
                wobbleClamp.setPosition(wobbleClampOpen);
                x = 0;
            }

            // Wobble Lift
            if (gamepad1.a && a == 0) {
                a = 1;
            } else if (!gamepad1.a && a == 1) {
                setWobbleMech(0.5, 1);
                a = 2;
            }  else if (gamepad1.a && a == 2) {
                a = 3;
            } else if (!gamepad1.a && a == 3) {
                setWobbleMech(0.5,2);
                a = 0;
            }

            if (gamepad1.b && b == 0) {
                b = 1;
            } else if (!gamepad1.b && b == 1) {
                setWobbleMech(0.5, 0);
                b = 0;
            }
        }
    }
}

