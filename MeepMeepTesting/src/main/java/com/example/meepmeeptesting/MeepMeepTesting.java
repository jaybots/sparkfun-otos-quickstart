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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(7, -61, Math.toRadians(90)))
                        .lineTo(new Vector2d(7,-33))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(35,-31,Math.toRadians(-90)),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(35,-21),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(47,-8),Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(47, -46), Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(47,-8),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(58,-20),Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(58,-46),Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(38,-65),Math.toRadians(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(2,-27, Math.toRadians(90)),Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(38,-65, Math.toRadians(-90)),Math.toRadians(-90))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}