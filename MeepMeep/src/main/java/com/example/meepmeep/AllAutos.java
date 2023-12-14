package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AllAutos {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity BlueRightL = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))

                                .splineToLinearHeading(new Pose2d(-33, 40, Math.toRadians(0)), Math.toRadians(100))
                                .lineToConstantHeading(new Vector2d(-32, 32.5))
                                .lineToConstantHeading(new Vector2d(-24, 32.5))
                                .lineToConstantHeading(new Vector2d(-32, 30))


                                // Play orange pixel
                                .lineToLinearHeading(new Pose2d(-37, 8, Math.toRadians(4)))
                                .lineToConstantHeading(new Vector2d(20, 10))
                                .splineToConstantHeading(new Vector2d(52, 40), Math.toRadians(0))

                                .build()
                );
        RoadRunnerBotEntity BlueRightM = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))

                                .lineToConstantHeading(new Vector2d(-36, 10))
                                .lineToConstantHeading(new Vector2d(-36, 36))
                                .lineToLinearHeading(new Pose2d(-37, 10, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(20, 10))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(53.3, 36))


                                .build()
                );
        RoadRunnerBotEntity BlueRightR = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(-90)))

                                .lineToConstantHeading(new Vector2d(-47, 60))
                                .lineToConstantHeading(new Vector2d(-47, 10))
                                .lineToConstantHeading(new Vector2d(-47, 40))
                                .lineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(2)))
                                .lineToConstantHeading(new Vector2d(20, 12))
                                .splineToConstantHeading(new Vector2d(53, 35), Math.toRadians(0))

                                .build()
                );

        RoadRunnerBotEntity RedLeftL = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(-47, -60))
                                .lineToConstantHeading(new Vector2d(-47, -10))
                                .lineToConstantHeading(new Vector2d(-47, -40))
                                .lineToLinearHeading(new Pose2d(-30, -10, Math.toRadians(2)))
                                .lineToConstantHeading(new Vector2d(20, -12))
                                .splineToLinearHeading(new Pose2d(53, -35, Math.toRadians(0)), Math.toRadians(0))

                                .build());


        RoadRunnerBotEntity RedLeftM = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))

                                .lineToConstantHeading(new Vector2d(-36, -10))
                                .lineToConstantHeading(new Vector2d(-36, -36))
                                .lineToLinearHeading(new Pose2d(-37, -10, Math.toRadians(0)))
                                .lineToConstantHeading(new Vector2d(20, -10))
                                .setReversed(true)
                                .lineToConstantHeading(new Vector2d(53.3, -36))



                                .build());


        RoadRunnerBotEntity RedLeftR = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))

                                .splineToLinearHeading(new Pose2d(-33, -40, Math.toRadians(0)), Math.toRadians(100))
                                .lineToConstantHeading(new Vector2d(-32, -32.5))
                                .lineToConstantHeading(new Vector2d(-24, -32.5))
                                .lineToConstantHeading(new Vector2d(-32, -30))


                                // Play orange pixel
                                .lineToLinearHeading(new Pose2d(-37, -8, Math.toRadians(4)))
                                .lineToConstantHeading(new Vector2d(20, -10))
                                .splineToConstantHeading(new Vector2d(52, -40), Math.toRadians(0))

                                .build());

        RoadRunnerBotEntity BlueLeftL = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))

                                // Knock the team prop out of the way
                                .lineToConstantHeading(new Vector2d(26, 60))
                                .lineToConstantHeading(new Vector2d(26, 10))
                                // Drop the purple pixel
                                .lineToConstantHeading(new Vector2d(26,40))
                                .waitSeconds(1)

                                // Play the Orange pixel
                                .lineToLinearHeading(new Pose2d(53, 40, Math.toRadians(0)))
                                .waitSeconds(.5)

                                .build()
                );
        RoadRunnerBotEntity BlueLeftM = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))

                                // Purple Pixel Drop
                                .lineToConstantHeading(new Vector2d(12,24))
                                .lineToConstantHeading(new Vector2d(12, 37))
                                .waitSeconds(1)

                                // Orange Pixel Drop
                                .lineToConstantHeading(new Vector2d(12, 39))
                                .lineToLinearHeading(new Pose2d(52, 36, Math.toRadians(0)))
                                .waitSeconds(.5)


                                .build()
                );
        RoadRunnerBotEntity BlueLeftR = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))

                                // Knock the Team Prop out of the way
                                .splineToLinearHeading(new Pose2d(9, 40, Math.toRadians(180.00)), Math.toRadians(100))
                                .lineToConstantHeading(new Vector2d(8, 32.5))
                                .lineToConstantHeading(new Vector2d(0, 32.5))

                                // Place the Purple Pixel
                                .lineToConstantHeading(new Vector2d(8, 30))
                                .waitSeconds(1)

                                // Place the Orange Pixel
                                .lineToLinearHeading(new Pose2d(53, 30, Math.toRadians(0)))

                                .build()
                );

        RoadRunnerBotEntity RedRightL = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                                // Knock the Team Prop out of the way
                                .splineToLinearHeading(new Pose2d(9, -40, Math.toRadians(180.00)), Math.toRadians(100))
                                .lineToConstantHeading(new Vector2d(8, -32.5))
                                .lineToConstantHeading(new Vector2d(0, -32.5))

                                // Place the Purple Pixel
                                .lineToConstantHeading(new Vector2d(8, -30))


                                // Place the Orange Pixel
                                .lineToLinearHeading(new Pose2d(53,-18,Math.toRadians(0)))

                                .build());


        RoadRunnerBotEntity RedRightM = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                                // Knock the Team Prop out of the way
                                .lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(90)))

                                // Place the purple Pixel
                                .lineToLinearHeading(new Pose2d(12, -37, Math.toRadians(90)))
                                .waitSeconds(1)

                                .lineToLinearHeading(new Pose2d(12, -39, Math.toRadians(90)))

                                // Place the Orange Pixel
                                .lineToLinearHeading(new Pose2d(52, -36, Math.toRadians(0)))
                                .waitSeconds(.5)



                                .build());


        RoadRunnerBotEntity RedRightR = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                                // Knock the team prop out of the way
                                .lineToConstantHeading(new Vector2d(26, -60))
                                .lineToConstantHeading(new Vector2d(26, -10))
                                // Drop the purple pixel
                                .lineToConstantHeading(new Vector2d(26, -40))
                                .waitSeconds(1)

                                // Play the Orange pixel
                                .lineToLinearHeading(new Pose2d(53, -40, Math.toRadians(0)))
                                .waitSeconds(.5)

                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                .addEntity(RedLeftL)
                .addEntity(RedLeftM)
                .addEntity(RedLeftR)
                .addEntity(BlueRightL)
                .addEntity(BlueRightM)
                .addEntity(BlueRightR)
                .addEntity(RedRightL)
                .addEntity(RedRightM)
                .addEntity(RedRightR)
                .addEntity(BlueLeftL)
                .addEntity(BlueLeftM)
                .addEntity(BlueLeftR)
                .start();
    }
}