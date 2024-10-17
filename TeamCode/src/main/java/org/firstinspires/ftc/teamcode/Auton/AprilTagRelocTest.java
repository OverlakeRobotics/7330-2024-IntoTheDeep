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

@Autonomous(name = "Blue Team Autonomous Close", group = "Autonomous")
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
                .splineTo(new Vector2d (-40.6, 40), 0);





        waitForStart();

        if (isStopRequested()) return;

        Action traj = traj1.build();

        Actions.runBlocking(traj);

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        AprilTagDetection detection = currentDetections.get(0);
        Pose2d reloc = new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw());
        drive.pose = reloc;

        TrajectoryActionBuilder traj2 = drive.actionBuilder(reloc)
                .splineTo(new Vector2d (-40.6, 20), 0);

        Action traja = traj2.build();
        Actions.runBlocking (traja);


    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(687.613, 687.613, 310.39, 250.104)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (true) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
}
