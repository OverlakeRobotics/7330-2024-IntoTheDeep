package org.firstinspires.ftc.teamcode.Auton;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.components.Intake;

@Config
@Autonomous(name = "Blue Team Autonomous Basket")
public class BlueAutonBasket extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Intake inta = new Intake(hardwareMap);

        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                .lineToY(50);
        Action tab2 = tab.fresh()
                .strafeTo(new Vector2d(55, 10))
                .build();

        if (isStopRequested()) return;

        Action ta = tab.build();

        Actions.runBlocking(new SequentialAction(
                ta,
                inta.intakeAuto(),
                tab2
        ));
    }
}
