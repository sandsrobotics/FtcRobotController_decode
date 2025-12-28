package org.firstinspires.ftc.teamcode.parts.artifact;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ArtifactDetectionPipeline extends OpenCvPipeline
{
    public enum ArtifactColor
    {
        NONE,
        GREEN,
        PURPLE
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar YELLOW = new Scalar(0, 255, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar BLOCK = new Scalar(76,166,40);
    static final Scalar WHITE = new Scalar(255,255,255);
    static final Scalar PURPLE = new Scalar(128, 0, 128);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final double leftTagLeftTopLeft = 50;
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,300);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(550,400);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(980,300); // was x=1080
    static final int tpREGION_WIDTH = 200;
    static final int tpREGION_HEIGHT = 300;

    Scalar rectangleColor = BLOCK;

    /* Points which actually define the sample region rectangles, derived from above values
     * Example of how points A and B work to define a rectangle
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + tpREGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + tpREGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + tpREGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + tpREGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + tpREGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + tpREGION_HEIGHT);

    Artifact[] artifacts = new Artifact[]{
            /* These are reversed to follow the camera. */
            new Artifact(ArtifactColor.NONE, null, region3_pointA, region3_pointB, 0),
            new Artifact(ArtifactColor.NONE, null, region2_pointA, region2_pointB, 0),
            new Artifact(ArtifactColor.NONE, null, region1_pointA, region1_pointB, 0)
    };

    /*
     * Working variables
     */
    Mat inputConv = new Mat();
    Mat extracted = new Mat();

//    Volatile since accessed by OpMode thread w/o synchronization
//    public volatile TeamPropPosition position = TeamPropPosition.NONE;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, inputConv, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(inputConv, extracted, 2);
    }

    void inputToSat(Mat input)
    {
        Imgproc.cvtColor(input, inputConv, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(inputConv, extracted, 1);
    }

    void inputToHsv(Mat input)
    {
        Imgproc.cvtColor(input, inputConv, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(inputConv, extracted, 0);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
//        inputToCb(firstFrame);
//        inputToSat(firstFrame);
        inputToHsv(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */

        // create the region mat for each artifact
        for (Artifact  artifact : artifacts) {
            artifact.submat = extracted.submat(new Rect(artifact.pta, artifact.ptb));
        }
    }

    @Override
    public Mat processFrame(Mat input)
    {
        //inputToCb(input);
//        inputToSat(input);
        inputToHsv(input);
        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */

        // Determine the color and status of artifacts in array
        for (Artifact  artifact : artifacts) {
            artifact.average = (int) Core.mean(artifact.submat).val[0];
            if (artifact.average > 60 && artifact.average < 80) {
                rectangleColor = GREEN;
                artifact.color = ArtifactColor.GREEN;
            }
            else if (artifact.average > 100 && artifact.average < 155) {
                rectangleColor = PURPLE;
                artifact.color = ArtifactColor.PURPLE;
            } else {
                rectangleColor = WHITE;
                artifact.color = ArtifactColor.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    artifact.pta, // First point which defines the rectangle
                    artifact.ptb, // Second point which defines the rectangle
                    rectangleColor, // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines
            Imgproc.putText(input, // Buffer to draw on
                    String.valueOf(artifact.average), // string
                    artifact.pta, // position point
                    Imgproc.FONT_HERSHEY_SIMPLEX,      // font face
                    4,                               // font scale
                    WHITE,             // Scalar object for color
                    4); // thickness
            }
        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    public static class Artifact{
        public ArtifactDetectionPipeline.ArtifactColor color;
        public Mat submat;
        public Point pta;
        public Point ptb;
        public int average;

        private Artifact(ArtifactDetectionPipeline.ArtifactColor color, Mat submat, Point pta, Point ptb, int average) {
            this.color = color;
            this.submat = submat;
            this.pta = pta;
            this.ptb = ptb;
            this.average = average;
        }
    }

    public Artifact[] getArtifactList() {
        return artifacts;
    }

    public int getArtifactCount() {
        int totalArtifacts = 0;
        for (Artifact  artifact : artifacts) {
            if (artifact.color != ArtifactColor.NONE) {
                totalArtifacts++;
            }
        }
        return totalArtifacts;
    }
}
