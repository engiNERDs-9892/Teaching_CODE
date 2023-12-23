package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ShortAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity BlueLeftL = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))

                                // Knock the team prop out of the way
                                .lineToLinearHeading(new Pose2d(20, 60, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(20, 10, Math.toRadians(-90)))

                                // Drop the purple pixel
                                .lineToLinearHeading(new Pose2d(20, 40, Math.toRadians(-90)))
                                .waitSeconds(2)


                                // Play the Orange pixel
                                .lineToLinearHeading(new Pose2d(53, 45, Math.toRadians(0)))
                                .waitSeconds(1.5)

                                .lineToLinearHeading(new Pose2d(48, 45, Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(48, 10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(62, 10, Math.toRadians(180)))


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

                                .splineToLinearHeading(new Pose2d(16.00, -37.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .setReversed(true)
                                // Orange Pixel
                                .splineToLinearHeading(new Pose2d(51, -42, Math.toRadians(-180.00)), Math.toRadians(0.00))
                                .waitSeconds(0.5)


                                .build());


        RoadRunnerBotEntity RedRightR = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                                // Knock the team prop out of the way


                                // Drop the purple pixel
                                .splineToLinearHeading(new Pose2d(20, -40, Math.toRadians(180)),Math.toRadians(180))

                                // Play the Orange pixel
                                .lineToLinearHeading(new Pose2d(53, -45, Math.toRadians(180)))

                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(40, -68, Math.toRadians(-180.00)), Math.toRadians(-90))
                                .lineToLinearHeading(new Pose2d(68, -68, Math.toRadians(180.00)))


                                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                //.addEntity(RedRightL)
                .addEntity(RedRightM)
                //.addEntity(RedRightR)
                //.addEntity(BlueLeftL)
                //.addEntity(BlueLeftM)
                //.addEntity(BlueLeftR)
                .start();
    }
}