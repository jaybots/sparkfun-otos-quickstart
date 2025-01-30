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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(7, -32, Math.PI/2))
                        .setTangent(-Math.PI/2)
                        .splineToSplineHeading(new Pose2d(35,-31,-Math.PI/2),Math.PI/2)
                        .splineToConstantHeading(new Vector2d(35,-21),Math.PI/2)
                        .splineToConstantHeading(new Vector2d(47,-11),-Math.PI/2)
                        .splineToConstantHeading(new Vector2d(47,-46),-Math.PI/2)
                        .splineToConstantHeading(new Vector2d(47,-40),-Math.PI/2)
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