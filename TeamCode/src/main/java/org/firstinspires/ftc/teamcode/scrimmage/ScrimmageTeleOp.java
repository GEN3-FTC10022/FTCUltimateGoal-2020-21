package org.firstinspires.ftc.teamcode.scrimmage;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOp: TeleOp")

public class ScrimmageTeleOp extends ScrimmageSuperclass {

    @Override
    public void runOpMode() {

        initialize();

        waitForStart();


        while (opModeIsActive()) {


            // Print Robot Orientation
            theta = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);



            telemetry.addData("Rotation (Radians): ", theta.firstAngle);
            telemetry.addData("Rotation (Degrees): ", Math.toDegrees(theta.firstAngle));


            double forward = -gamepad1.left_stick_y;
            // double forward = -1;
            double right = gamepad1.left_stick_x;
            double clockwise = gamepad1.right_stick_x;

            // If theta is measured clockwise from zero reference
            if (theta.firstAngle <= 0) {

                temp = forward * Math.cos(-theta.firstAngle) + right * Math.sin(-theta.firstAngle);
                right = -forward * Math.sin(-theta.firstAngle) + right * Math.cos(-theta.firstAngle);
                forward = temp;
            }

            // If theta is measured counterclockwise from zero reference

            if (theta.firstAngle > 0) {

                // Theta is reversed to account for IMU measurement
                temp = forward * Math.cos(theta.firstAngle) - right * Math.sin(theta.firstAngle);
                right = forward * Math.sin(theta.firstAngle) + right * Math.cos(theta.firstAngle);
                forward = temp;
            }

            flpower = forward + clockwise + right;
            frpower = forward - clockwise - right;
            blpower = forward + clockwise - right;
            brpower = forward - clockwise + right;

            double max = Math.abs(flpower);

            if (Math.abs(frpower) > max) {

                max = Math.abs(frpower);
            }

            if (Math.abs(blpower) > max) {

                max = Math.abs(blpower);
            }

            if (Math.abs(brpower) > max) {

                max = Math.abs(brpower);
            }

            if (max > 1) {

                flpower /= max;
                frpower /= max;
                blpower /= max;
                brpower /= max;
            }


            frontLeft.setPower(-flpower);
            frontRight.setPower(-frpower);
            backLeft.setPower(-blpower);
            backRight.setPower(brpower);








            /*

            // DRIVETRAIN

            double lefty = -gamepad1.left_stick_y;
            double leftx = gamepad1.left_stick_x;
            double rightx = gamepad1.right_stick_x;

            // Joy stick deadzones
            if (Math.abs(lefty) < 0.2)
                lefty = 0;
            if (Math.abs(leftx) < 0.2)
                leftx = 0;
            if (Math.abs(rightx) < 0.2)
                rightx = 0;

            // Motor powers are set to the power of 3 so that the drivetrain motors accelerates
            // exponentially instead of linearly

            double flpower = Math.pow((lefty + leftx + rightx), 3);
            double blpower = Math.pow((lefty - leftx + rightx), 3);
            double frpower = Math.pow((lefty - leftx - rightx), 3);
            double brpower = Math.pow((lefty + leftx - rightx), 3);

            // Right Trigger speed modifier

            if (gamepad1.right_trigger > 0.5){
                flpower /= 3;
                blpower /= 3;
                frpower /= 3;
                brpower /= 3;
            }


           // Set Motor Powers
            frontLeft.setPower(flpower);
            backLeft.setPower(blpower);
            frontRight.setPower(frpower);
            backRight.setPower(brpower);

            */

        }
    }
}

