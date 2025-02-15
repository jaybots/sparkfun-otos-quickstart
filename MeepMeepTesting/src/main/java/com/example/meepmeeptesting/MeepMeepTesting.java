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
                        .setTangent(-Math.PI/2)
                        .splineTo(new Vector2d(-19,-35),Math.PI)
                        .splineTo(new Vector2d(-37,-8),Math.toRadians(85))
                        .setTangent(Math.toRadians(265))
                        .splineTo(new Vector2d(-42,-25),Math.toRadians(265))
                        .splineTo(new Vector2d(-44,-55),Math.toRadians(265))
                        .setTangent(Math.toRadians(85))
                        .splineTo(new Vector2d(-46,-8),Math.toRadians(85))
                        .setTangent(Math.toRadians(265))
                        .splineTo(new Vector2d(-58,-45),Math.toRadians(270))
                        .setTangent(Math.toRadians(90))
                        .splineTo(new Vector2d(-50,0),Math.toRadians(90))
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-60,-10),Math.toRadians(270))
                        .splineTo(new Vector2d(-60,-42),Math.toRadians(270))
                        .setTangent(Math.toRadians(90))
                        .splineTo(new Vector2d(-30,-9),Math.toRadians(0))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}