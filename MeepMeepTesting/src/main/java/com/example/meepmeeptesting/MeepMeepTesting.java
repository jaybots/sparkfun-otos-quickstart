package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(7, -34, Math.PI/2))
                                .strafeTo(new Vector2d(24,-34))
                                .splineToConstantHeading(new Vector2d(38,-10),Math.PI/2)
                                .strafeTo(new Vector2d(48,-10))
                                .lineTo(new Vector2d(48, -50))
                                .lineTo(new Vector2d(48, -40))
                                .turn(Math.PI)
//                        .splineTo(new Vector2d(35.86, -32.11), Math.toRadians(83.07))
//                        .splineTo(new Vector2d(35.86, -9.64), Math.toRadians(-35.22))
//                        .splineTo(new Vector2d(42.98, -54.59), Math.toRadians(270.00))
                                .build());



        //.strafeRight(19)
                        //.splineToConstantHeading(new Vector2d(44,-10),0)
                        //.back(40)
                        //.forward(5)
                        //.turn(Math.PI)
                        //.forward(15)
                        //.lineToLinearHeading(new Pose2d(9,-35,Math.PI/2))
                        //.build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}