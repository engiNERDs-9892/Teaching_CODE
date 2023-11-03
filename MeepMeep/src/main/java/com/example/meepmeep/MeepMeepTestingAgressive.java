package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingAgressive {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity BlueRight = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(270.00)))

                                        // Spline to drop of Purple Pixel
                                        .splineTo(new Vector2d(-36.00, 32.00), Math.toRadians(270.00))
                                        .setReversed(true)

                                        // Spline to backboard - Orange Pixel drop off (1)
                                        .splineToLinearHeading(new Pose2d(-26.92, 35.50, Math.toRadians(360.00)), Math.toRadians(360.00))
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(52.00, 35.00, Math.toRadians(360.00)), Math.toRadians(360.00))
                                        .setReversed(true)

                                        // Spline to far Stack (1)
                                        .splineToLinearHeading(new Pose2d(-17.28, 59.20, Math.toRadians(360.00)), Math.toRadians(180.00))
                                        .splineToLinearHeading(new Pose2d(-63.23, 11.4, Math.toRadians(180.00)), Math.toRadians(180.00))

                                        // Spline to Backboard (2)
                                        .splineToLinearHeading(new Pose2d(-27.10, 35.50, Math.toRadians(180.00)), Math.toRadians(360.00))
                                        .splineToLinearHeading(new Pose2d(52.00, 35.00, Math.toRadians(360.00)), Math.toRadians(360.00))

                                        // Spline to far Stack (2)
                                        .splineToLinearHeading(new Pose2d(-17.28, 59.20, Math.toRadians(360.00)), Math.toRadians(180.00))
                                        .splineToLinearHeading(new Pose2d(-63.23, 11.4, Math.toRadians(180.00)), Math.toRadians(180.00))

                                        // Spline to Backboard (3)
                                        .splineToLinearHeading(new Pose2d(-27.10, 35.50, Math.toRadians(180.00)), Math.toRadians(360.00))
                                        .splineToLinearHeading(new Pose2d(52.00, 35.00, Math.toRadians(360.00)), Math.toRadians(360.00))

                                        .build()
                );


        RoadRunnerBotEntity BlueLeft = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90.00)))

                                .splineTo(new Vector2d(12.00, 32.00), Math.toRadians(270.00))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(52.00, 35.00, Math.toRadians(540.00)), Math.toRadians(360.00))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-22.93, 59.36), Math.toRadians(194.04))
                                .splineToConstantHeading(new Vector2d(-61.62, 11.5), Math.toRadians(180.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-29.35, 32), Math.toRadians(375.24))
                                .splineToConstantHeading(new Vector2d(52.00, 35.00), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-22.93, 59.36), Math.toRadians(194.04))
                                .splineToConstantHeading(new Vector2d(-61.62, 11.5), Math.toRadians(180.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-29.35, 32), Math.toRadians(375.24))
                                .splineToConstantHeading(new Vector2d(52.00, 35.00), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-22.93, 59.36), Math.toRadians(194.04))
                                .splineToConstantHeading(new Vector2d(-61.62, 11.5), Math.toRadians(180.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-29.35, 32), Math.toRadians(375.24))
                                .splineToConstantHeading(new Vector2d(52.00, 35.00), Math.toRadians(0.00))
                                .setReversed(false)

                                .build()
                );

        RoadRunnerBotEntity RedRight = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))

                                .splineTo(new Vector2d(12.00, -32.00), Math.toRadians(90.00))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(52.00, -35.00, Math.toRadians(-180.00)), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-22.93, -59.36), Math.toRadians(165.96))
                                .splineToConstantHeading(new Vector2d(-61.62, -11.5), Math.toRadians(180.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-29.35, -32), Math.toRadians(-15.24))
                                .splineToConstantHeading(new Vector2d(52.00, -35.00), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-22.93, -59.36), Math.toRadians(165.96))
                                .splineToConstantHeading(new Vector2d(-61.62, -11.5), Math.toRadians(180.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-29.35, -32), Math.toRadians(-15.24))
                                .splineToConstantHeading(new Vector2d(52.00, -35.00), Math.toRadians(0.00))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-22.93, -59.36), Math.toRadians(165.96))
                                .splineToConstantHeading(new Vector2d(-61.62, -11.5), Math.toRadians(180.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-29.35, -32), Math.toRadians(-15.24))
                                .splineToConstantHeading(new Vector2d(52.00, -35.00), Math.toRadians(0.00))
                                .setReversed(false)

                                .build()
                );

        RoadRunnerBotEntity RedLeft = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))

                                .splineTo(new Vector2d(-36.00, -32.00), Math.toRadians(90.00))
                                .setReversed(true)
                                .splineTo(new Vector2d(-32.74, -38.97), Math.toRadians(9.75))
                                .splineToLinearHeading(new Pose2d(52.00, -35.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .splineToConstantHeading(new Vector2d(-40.29, -58.60), Math.toRadians(170.73))
                                .splineToConstantHeading(new Vector2d(-63.13, -11.80), Math.toRadians(180.00))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-29.35, -32), Math.toRadians(-15.24))
                                .splineToConstantHeading(new Vector2d(52.00, -35.00), Math.toRadians(0.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-40.29, -58.60), Math.toRadians(170.73))
                                .splineToConstantHeading(new Vector2d(-63.13, -11.80), Math.toRadians(180.00))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-29.35, -32), Math.toRadians(-15.24))
                                .splineToConstantHeading(new Vector2d(52.00, -35.00), Math.toRadians(0.00))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-40.29, -58.60), Math.toRadians(170.73))
                                .splineToConstantHeading(new Vector2d(-63.13, -11.80), Math.toRadians(180.00))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-29.35, -32), Math.toRadians(-15.24))
                                .splineToConstantHeading(new Vector2d(52.00, -35.00), Math.toRadians(0.00))

                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(RedLeft)
                .addEntity(RedRight)
                .addEntity(BlueLeft)
                //.addEntity(BlueRight)
                .start();
    }
}