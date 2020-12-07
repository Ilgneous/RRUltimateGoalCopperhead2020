package org.firstinspires.ftc.teamcode.Vision;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;
import org.firstinspires.ftc.teamcode.oldexcode.Vision.BitMapVision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.BlockingQueue;

import static android.graphics.Color.red;

@TeleOp(name="bitMapTests", group="Pushbot")
public class bitMapTests extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    private Parameters parameters;
    private CameraDirection CAMERA_CHOICE = CameraDirection.BACK;

    private final int RED_THRESHOLD = 25;
    private final int GREEN_THRESHOLD = 25;
    private final int BLUE_THRESHOLD = 25;

    private final double widthFactor =  1280.0/3264;
    private final double heightFactor =  720.0/1836;

    private static final String VUFORIA_KEY = "Aad5SFz/////AAABmT+V+odOO0Lcr/j0iQ+cxkFwNC+TZ2mUCftLAt5wXY/BCUwwq4iU84o/Q15qndcAU2JPL+SN/qG+8GE9j0fDBHGkUHqAOBhu41XjysYMyF+kBeicTqdwfzaUzT5NLSvAU8aOZ+oIyQu+KFBoHVGylT7Yf6ASJw9gX34kck2ECReLzij/3gORcXvFFtXm/rGIyQGlxPGOXDv0ZnYmurQ79fH6fnArDP6Ylcc9QYQuPInhQ7BKzhiicPZhciwnKfZfa3CH4zu8zSWajDLDQmaj3iZxnkVsXac6XCk2cYMD37svmgGqI78M4bAcRUfOpPqGtqz+89jC8kUmCip8OSdmGO7ArKzBPHF3sctfMxurkuG4";

    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frame;

    public static String bitmapSkyStonePosition;

    public String skyStonePos = "";
    BitMapVision bm1 = null;


    //WebcamName webcamName = null;

    @Override
    public void runOpMode() throws InterruptedException {





        waitForStart();

        bm1 = new BitMapVision(this);
        skyStonePos = bm1.findBlueSkystones();
        telemetry.addData("stone", skyStonePos);
        telemetry.update();

        //getBitmap();
        //String pos =
        //telemetry.addData("pos", pos);
        //telemetry.update();

        //sample();
        sleep(5000);
        //telemetry.addData("width", bm)


    }






    public Bitmap vufConvertToBitmap(Frame frame) { return vuforia.convertFrameToBitmap(frame); }


}
