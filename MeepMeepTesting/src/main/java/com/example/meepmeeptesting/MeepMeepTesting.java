package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                //Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-7, -61, Math.toRadians(90)))

                        .lineTo(new Vector2d(-7,-30))
                        .splineToConstantHeading(new Vector2d(-14,-40),Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-33,-21),Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-47,-8),-Math.PI/2)
                        .splineToConstantHeading(new Vector2d(-47, -56), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-47, -6), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-55, -18), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-55, -52), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-55, -6), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-60, -18), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-60, -44), -Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-60, -24), -Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(-20,-8,Math.PI),0)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}