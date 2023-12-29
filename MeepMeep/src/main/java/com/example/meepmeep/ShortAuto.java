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

                                // 1.70 Seconds to drive to the Spike Mark (PURPLE PIXEL | LEFT CLAW)
                                .lineToLinearHeading(new Pose2d(22, 43, Math.toRadians(-90)))
                                .waitSeconds(0.5)


                                // 2.40 Seconds to drive to the Backboard (ORANGE PIXEL | RIGHT CLAW)
                                .lineToLinearHeading(new Pose2d(53, 45, Math.toRadians(180)))
                                .waitSeconds(0.5)


                                // 1.66 Seconds to drive to Park (RESET CLAWS / Wrist - *POSSIBLY EXTEND INTAKE*)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(45, 55, Math.toRadians(180.00)), Math.toRadians(-270))
                                .splineToLinearHeading(new Pose2d(68, 68, Math.toRadians(180.00)), Math.toRadians(-270))


                                .build()
                );
        RoadRunnerBotEntity BlueLeftM = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))

                                // 2.35 Seconds to drive to the Spike Mark (PURPLE PIXEL | LEFT CLAW)
                                .splineToLinearHeading(new Pose2d(16.00, 37.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .setReversed(true)
                                .waitSeconds(0.5)

                                // 2 Seconds to drive to the Backboard (ORANGE PIXEL | RIGHT CLAW)
                                .splineToLinearHeading(new Pose2d(51, 42, Math.toRadians(-180.00)), Math.toRadians(0.00))
                                .waitSeconds(0.5)

                                // 1.79 Seconds to drive to Park (RESET CLAWS / Wrist - *POSSIBLY EXTEND INTAKE*)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(45, 55, Math.toRadians(-180.00)), Math.toRadians(-270))
                                .splineToLinearHeading(new Pose2d(68, 68, Math.toRadians(-180.00)), Math.toRadians(-270))



                                .build()
                );
        RoadRunnerBotEntity BlueLeftR = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.50)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(-90)))

                                // 1.85 Seconds to drive to the Spike Mark (PURPLE PIXEL | LEFT CLAW)
                                .splineToLinearHeading(new Pose2d(6.00, 40.00, Math.toRadians(-114.44)), Math.toRadians(120.00))
                                .waitSeconds(.5)

                                // 2.40 Seconds to drive to the Backboard (ORANGE PIXEL | RIGHT CLAW)
                                .lineToLinearHeading(new Pose2d(53, 24, Math.toRadians(180.00)))
                                .waitSeconds(.5)

                                // 2.67 Seconds to drive to Park (RESET CLAWS / Wrist - *POSSIBLY EXTEND INTAKE*)
                                .splineToLinearHeading(new Pose2d(45, 8.00, Math.toRadians(180)), Math.toRadians(180.00))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(68.00, 8.00, Math.toRadians(180)), Math.toRadians(180.00))


                                .build()
                );

        RoadRunnerBotEntity RedRightL = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))

                                // 1.85 Seconds to drive to the Spike Mark (PURPLE PIXEL | LEFT CLAW)
                                .splineToLinearHeading(new Pose2d(6.00, -40.00, Math.toRadians(114.44)), Math.toRadians(120.00))
                                .waitSeconds(.5)

                                // 1.50 Seconds to drive to the Backboard (ORANGE PIXEL | RIGHT CLAW)
                                .lineToLinearHeading(new Pose2d(53, -24, Math.toRadians(180.00)))
                                .waitSeconds(.5)


                                // 2.67 Seconds to drive to Park (RESET CLAWS / Wrist - *POSSIBLY EXTEND INTAKE*)
                                .splineToLinearHeading(new Pose2d(45, -8.00, Math.toRadians(180)), Math.toRadians(180.00))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(68.00, -8.00, Math.toRadians(180)), Math.toRadians(180.00))




                                .build());


        RoadRunnerBotEntity RedRightM = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))

                                // 2.35 Seconds to drive to the Spike Mark (PURPLE PIXEL | LEFT CLAW)
                                .splineToLinearHeading(new Pose2d(16.00, -37.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                                .setReversed(true)
                                .waitSeconds(0.5)


                                // 2 Seconds to drive to the Backboard (ORANGE PIXEL | RIGHT CLAW)
                                .splineToLinearHeading(new Pose2d(51, -42, Math.toRadians(-180.00)), Math.toRadians(0.00))
                                .waitSeconds(0.5)


                                // 1.79 Seconds to drive to Park (RESET CLAWS / Wrist - *POSSIBLY EXTEND INTAKE*)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(45, -55, Math.toRadians(-180.00)), Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(68, -68, Math.toRadians(-180.00)), Math.toRadians(-90))




                                .build());


        RoadRunnerBotEntity RedRightR = new DefaultBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(60), Math.toRadians(60), 16.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))

                                // 1.70 Seconds to drive to the Spike Mark (PURPLE PIXEL | LEFT CLAW)
                                .lineToLinearHeading(new Pose2d(25, -43, Math.toRadians(90)))
                                .waitSeconds(0.5)


                                // 2.40 Seconds to drive to the Backboard (ORANGE PIXEL | RIGHT CLAW)
                                .lineToLinearHeading(new Pose2d(53, -45, Math.toRadians(180)))
                                .waitSeconds(0.5)


                                // 1.66 Seconds to drive to Park (RESET CLAWS / Wrist - *POSSIBLY EXTEND INTAKE*)
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(45, -55, Math.toRadians(180.00)), Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(68, -68, Math.toRadians(180.00)), Math.toRadians(-90))


                                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                .addEntity(RedRightL)
                //.addEntity(RedRightM)  (DONE)
                //.addEntity(RedRightR)  (DONE)
                //.addEntity(BlueLeftL)
                //.addEntity(BlueLeftM)
                .addEntity(BlueLeftR)
                .start();
    }
}