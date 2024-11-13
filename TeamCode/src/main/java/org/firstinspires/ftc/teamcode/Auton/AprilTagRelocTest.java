package org.firstinspires.ftc.teamcode.Auton;

// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

// Non-RR imports
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
        import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


public class AprilTagRelocTest extends LinearOpMode {

    private SparkFunOTOSDrive drive;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private final Position CAMERA_POSITION = new Position(DistanceUnit.INCH,
            -4, -7, 17.4, 16);
    private final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 180, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        initAprilTag();


        Pose2d initialPose = new Pose2d(-33.6, 59.3, Math.toRadians(180));
        drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d (-40.6, 40), 180);

        waitForStart();

        if (isStopRequested()) return;

        Action traja = traj1.build();

        Actions.runBlocking(traja);

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        AprilTagDetection detection = currentDetections.get(0);
        Pose2d reloc = new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw());
        drive.pose = reloc;

        TrajectoryActionBuilder traj2 = drive.actionBuilder(reloc)
                .splineTo(new Vector2d (-40.6, 20), 180);

        Action trajb = traj2.build();
        Actions.runBlocking (trajb);


    }

    private void initAprilTag() {


        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                .setLensIntrinsics(687.613, 687.613, 310.39, 250.104)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
}
