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
        Pose2d initialPose = new Pose2d(37, 62, Math.toRadians(0));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);



        TrajectoryActionBuilder t1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(37, 55))
                .strafeToLinearHeading(new Vector2d(50.5, 55.5), Math.toRadians(47));

//                .strafeTo(new Vector2d(-51,-58))
//                .turn(Math.toRadians(28));
        // Outtake Block
        TrajectoryActionBuilder t2 = t1.fresh()
                .strafeToLinearHeading(new Vector2d(55, 55), Math.toRadians(270));
        //Intake Block
        TrajectoryActionBuilder t3 = t2.fresh()
                .strafeTo(new Vector2d(-48.3,-38));
        //Arm up
        TrajectoryActionBuilder t4 = t3.fresh()
                .turn(Math.toRadians(136))
                .strafeTo(new Vector2d(-55.4,-55.2));
        //place block

        if (isStopRequested()) return;

        Action a1 = t1.build();
        Action a2 = t2.build();
        Action a3 = t3.build();
        Action a4 = t4.build();

        waitForStart();


        Actions.runBlocking(new SequentialAction(
                a1,
                intake.armToHighBasketAuto(),
                intake.extendAuto(),
                intake.outtakeAuto(),
                intake.contractAuto(),
                intake.armToFloorAuto()
                //a2
        ));


        //intake.outtake();

//        Actions.runBlocking(new ParallelAction(
//                a3,
//                intake.intakeAuto()
//        ));
//        Actions.runBlocking(new SequentialAction(
//                intake.armToLiftAuto(),
//                a4,
//                intake.armToHighBasketAuto(),
//                intake.outtakeAuto()
//        ));
    }
}
