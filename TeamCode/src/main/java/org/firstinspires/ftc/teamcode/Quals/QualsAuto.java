package org.firstinspires.ftc.teamcode.Quals;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Quals.QualsSuperclass;

@Autonomous (name = "Auto: Qual 1")
/*
 * This is a red side autonomous, but since there is only one side
 * for now, it's just labelled "auto."
 *
 * I believe our field is blue color, but red set up (remote field set up, per the game manual).
 */
public class QualsAuto extends QualsSuperclass {

    @Override
    public void runOpMode() {

        // Robot starts with one wobble goal preloaded, in a start line, wobble mech
        // facing away from the wall

        initialize();
        initializeVuforia();

        // Scan vuforia & assign to target_zone_goal using:
        //vuforiaScanTarget();
        // or
        //vuforiaScanPixel(true);               //set to false after testing
        String target_zone_goal = "A";          // A is closest to starting position, C is farthest

        // Deliver to target zone goal A, the closest one
        if (target_zone_goal == "A") {

            // deliver to zone goal A, the closest one
            forward(0.3, 30);               // all of these numbers need to be adjusted
            strafeRight(0.3, 12);           // all on this line & before

            //open wobble mech

            // return to get the next wobble
            backward(0.3, 30);
            strafeLeft(0.3, 12);

            //close wobble mech on the wobble goal

            //deliver to zone goal A
            forward(0.3, 30);
            strafeRight(0.3, 12);

            // move to the be positioned in the launch line
            backward(0.3, 0);

        } else if (target_zone_goal == "B") {

            // add the stuff from goal A and adjust numbers

        } else if (target_zone_goal == "C") {

            // add the stuff from goal A and adjust numbers

        }

    }

}