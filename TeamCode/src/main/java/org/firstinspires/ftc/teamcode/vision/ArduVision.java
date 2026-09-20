package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Reads the Hive's AprilTag cluster to tell which Cell is currently facing up,
 * or TIPPING if neither/both clusters are cleanly visible (mid-flip / no lock).
 * ID-only — never configured for pose, so no lens calibration needed and no
 * pose math runs per frame. Camera is assumed physically rotated for vertical
 * FOV; do NOT set a software rotation here (see notes — SENSOR_NATIVE stays default).
 */
@Configurable
public class ArduVision {

    public enum HiveState { CELL_A_UP, CELL_B_UP, TIPPING }

    // TODO: pull exact tag IDs per Cell from the Competition Manual's field
    // AprilTag assignment table before trusting this in a match.
    public static int[] CELL_A_TAG_IDS = {};
    public static int[] CELL_B_TAG_IDS = {};

    public static double MIN_DECISION_MARGIN = 35;
    public static int DECIMATION = 3;
    public static int RES_WIDTH = 320;
    public static int RES_HEIGHT = 240;

    private final AprilTagProcessor aprilTag;
    private final VisionPortal portal;
    private volatile HiveState lastState = HiveState.TIPPING;

    public ArduVision(HardwareMap hw, String webcamName) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                // no setTagLibrary(sized tags), no setLensIntrinsics -> ID-only, no pose math
                .build();
        aprilTag.setDecimation(DECIMATION);

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(RES_WIDTH, RES_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .build();
    }


    public HiveState update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        int aVotes = 0, bVotes = 0;
        for (AprilTagDetection d : detections) {
            if (d.decisionMargin < MIN_DECISION_MARGIN) continue;
            if (contains(CELL_A_TAG_IDS, d.id)) aVotes++;
            else if (contains(CELL_B_TAG_IDS, d.id)) bVotes++;
        }
        if (aVotes > 0 && bVotes == 0) lastState = HiveState.CELL_A_UP;
        else if (bVotes > 0 && aVotes == 0) lastState = HiveState.CELL_B_UP;
        else lastState = HiveState.TIPPING;
        return lastState;
    }

    public HiveState getState() { return lastState; }

    public void setEnabled(boolean enabled) {
        portal.setProcessorEnabled(aprilTag, enabled);
    }

    public void close() { portal.close(); }

    private static boolean contains(int[] ids, int id) {
        for (int i : ids) if (i == id) return true;
        return false;
    }
}