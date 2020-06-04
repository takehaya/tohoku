package jp.jaxa.iss.kibo.rpc.tohoku;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.google.astrozxing.BinaryBitmap;
import com.google.astrozxing.DecodeHintType;
import com.google.astrozxing.LuminanceSource;
import com.google.astrozxing.RGBLuminanceSource;
import com.google.astrozxing.Reader;
import com.google.astrozxing.common.HybridBinarizer;
import com.google.astrozxing.qrcode.QRCodeReader;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */


public class MainService extends KiboRpcService {
    public static final Boolean DOJAXA = true;
    public static final String LOGTAG = "TohokuKibo";
    public static final String EPOCK_UNIQUE_STR = UUID.randomUUID().toString().substring(4);

    private Mat cameraMatrix;
    private Mat distorsionMatrix;
    private final float arucoMarkerLength = 0.05f;//Length of one side
    private final float arucoToTargetDist = 0.2f;// 0.2m

    private final float[] NavCamvec = new float[]{-0.0422f, -0.117f, -0.0826f};//xyz
    private final float[] HazCamvec = new float[]{0.0352f, -0.1328f, -0.0826f};
    private final float[] Laservec = new float[]{0.0572f, -0.1302f, -0.1111f};

    private void cameraMatInit(){
        cameraMatrix = new Mat(3,3,CvType.CV_32FC1);
        distorsionMatrix = new Mat();
        // cameraMatrix will be of the form
        // | Fx 0  Cx |
        // | 0  Fy Cy |
        // | 0  0   1 |
        double[] cameraArray =  {
                344.173397, 0.000000, 630.793795,
                0.000000, 344.277922, 487.033834,
                0.000000, 0.00000000, 1.00000000,
        };

        cameraMatrix.put(0,0, cameraArray);

        double[] distArray =  {
                -0.152963, 0.017530, -0.001107, -0.000210, 0.000000,
        };

        distorsionMatrix.put(0,0, distArray);
    }

    @Override
    protected void runPlan1() {
        runPlan2();
//        runPlan3();

    }

    @Override
    protected void runPlan2() {
        Log.i(LOGTAG, "apijudgeSendStart");
        cameraMatInit();

        api.judgeSendStart();
        String p1_1_con[] = new String[]{"", ""};
        String p1_2_con[] = new String[]{"", ""};
        String p1_3_con[] = new String[]{"", ""};
        String p2_1_con[] = new String[]{"", ""};
        String p2_2_con[] = new String[]{"", ""};
        String p2_3_con[] = new String[]{"", ""};

        double px3 = 0,py3 = 0,pz3=0,qx3 = 0,qy3=0,qz3=0;
        int arv = 0;
        final int LOOPSIZE = 10;
        Vec3 robotpos = new Vec3();
        WrapQuaternion robotqt = new WrapQuaternion();

        double distance = 0.12;
        double inc = 0.00;
        Vec3 road1_1_v = new Vec3(11.15, -4.8, 4.55);
        Vec3 target1_1_v = new Vec3(11.5 - distance, -5.7, 4.5);
        Vec3 target1_2_v = new Vec3(11, -6, 5.55 - distance - 0.1);
        Vec3 target1_3_v = new Vec3(11, -5.5, 4.33 + distance);

        WrapQuaternion road1_1_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion target1_3_q = new WrapQuaternion(0, 0.707f, 0, 0.707f);
        WrapQuaternion target1_1_q = new WrapQuaternion(0, 0, 0, -1);
//        WrapQuaternion target1_2_q = new WrapQuaternion(0, -0.707f, 0, 0.707f);
        WrapQuaternion target1_2_q = new WrapQuaternion(0.707f, 0, 0.707f, 0);

        Vec3 road2_1_v = new Vec3(10.5, -6.45, 5.1);
        Vec3 road2_2_v = new Vec3(11.35, -7.3, 4.9);
        Vec3 target2_1_v = new Vec3(10.30 + distance, -7.5, 4.7);
        Vec3 target2_2_v = new Vec3(11.5 - distance, -8, 5);
        Vec3 target2_3_v = new Vec3(11, -7.7, 5.55 - distance);

        WrapQuaternion road2_1_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion road2_2_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion target2_1_q = new WrapQuaternion(0, 0, 1,0);
        WrapQuaternion target2_2_q = new WrapQuaternion(0, 0, 0,-1);
//        WrapQuaternion target2_3_q = new WrapQuaternion(0, -0.707f, 0, 0.707f);
        WrapQuaternion target2_3_q = new WrapQuaternion(0.707f, 0, 0.707f, 0);

        //moveTo(road1_1_v, road1_1_q);
        int loopCounter = 0;

        String p1_3 = "";
        p1_3 = scanBarcodeMoveTo(target1_3_v, target1_3_q, QRInfoType.PosZ);
        if (!p1_3.equals("error")) {
            p1_3_con = p1_3.split(", ");
            pz3 = Double.parseDouble(p1_3_con[1]);
        }
        while (!p1_3_con[0].equals("pos_z") && loopCounter < LOOPSIZE){
            Result.Status res = moveTo(target1_3_v.add(new Vec3(0, 0, -inc)), target1_3_q);
            if(res == null || res!=Result.Status.OK){
                res = relativeMoveTo(0,0,-0.15,0,0,0,1);
                if(res == null || res!=Result.Status.OK){
                    res = moveTo(10.95, -3.95, 4.65,0,0,0,1);
                }
            }

            p1_3 = scanOnecBarcode(QRInfoType.PosZ);
            if (!p1_3.equals("error")) {
                p1_3_con = p1_3.split(", ");
                pz3 = Double.parseDouble(p1_3_con[1]);
            }
            loopCounter++;
        }
        loopCounter = 0;
        Log.d(LOGTAG, "p1_3 = " + p1_3);



        String p1_1 = "";
        p1_1 = scanBarcodeMoveTo(target1_1_v, target1_1_q, QRInfoType.PosX);
        if (!p1_1.equals("error")) {
            p1_1_con = p1_1.split(", ");
            px3 = Double.parseDouble(p1_1_con[1]);
        }
        while (!p1_1_con[0].equals("pos_x") && loopCounter < LOOPSIZE){
            moveTo(target1_1_v.add(new Vec3(inc, 0, 0)), target1_1_q);
            p1_1 = scanOnecBarcode(QRInfoType.PosX);
            if (!p1_1.equals("error")) {
                p1_1_con = p1_1.split(", ");
                px3 = Double.parseDouble(p1_1_con[1]);
            }
            loopCounter++;
        }
        loopCounter = 0;
        Log.d(LOGTAG, "p1_1 = " + p1_1);


        String p1_2  = "";
        p1_2 = scanBarcodeMoveTo(target1_2_v, target1_2_q, QRInfoType.PosY);
        if (!p1_2.equals("error")) {
            p1_2_con = p1_2.split(", ");
            py3 = Double.parseDouble(p1_2_con[1]);
        }
        while (!p1_2_con[0].equals("pos_y") && loopCounter < LOOPSIZE){
            moveTo(target1_2_v.add(new Vec3(0, 0, inc)), target1_2_q);
            p1_2 = scanOnecBarcode(QRInfoType.PosY);
            if (!p1_2.equals("error")) {
                p1_2_con = p1_2.split(", ");
                py3 = Double.parseDouble(p1_2_con[1]);
            }
            loopCounter++;
        }
        loopCounter = 0;
        Log.d(LOGTAG, "p1_2 = " + p1_2);

        moveTo(road2_1_v, road2_1_q);
        moveTo(road2_2_v, road2_2_q);

        String p2_1  = "";
        p2_1 = scanBarcodeMoveTo(target2_1_v, target2_1_q, QRInfoType.QuaX);
        if (!p2_1.equals("error")) {
            p2_1_con = p2_1.split(", ");
            qx3 = Double.parseDouble(p2_1_con[1]);
        }
        while (!p2_1_con[0].equals("qua_x") && loopCounter < LOOPSIZE){
            moveTo(target2_1_v.add(new Vec3(-inc, 0,0)), target2_1_q);
            p2_1 = scanOnecBarcode(QRInfoType.QuaX);
            if (!p2_1.equals("error")) {
                p2_1_con = p2_1.split(", ");
                qx3 = Double.parseDouble(p2_1_con[1]);
            }
            loopCounter++;
        }
        loopCounter = 0;
        Log.d(LOGTAG, "p2_1 = " + p2_1);

        String p2_2  = "";
        p2_2 = scanBarcodeMoveTo(target2_2_v, target2_2_q, QRInfoType.QuaY);
        if (!p2_2.equals("error")) {
            p2_2_con = p2_2.split(", ");
            qy3 = Double.parseDouble(p2_2_con[1]);
        }
        while (!p2_2_con[0].equals("qua_y") && loopCounter < LOOPSIZE){
            moveTo(target2_2_v.add(new Vec3(inc, 0,0)), target2_2_q);
            p2_2 = scanOnecBarcode(QRInfoType.QuaY);
            if (!p2_2.equals("error")) {
                p2_2_con = p2_2.split(", ");
                qy3 = Double.parseDouble(p2_2_con[1]);
            }
            loopCounter++;
        }
        loopCounter = 0;
        Log.d(LOGTAG, "p2_2 = " + p2_2);


        String p2_3  = "";
        p2_3 = scanBarcodeMoveTo(target2_3_v, target2_3_q, QRInfoType.QuaZ);
        if (!p2_3.equals("error")) {
            p2_3_con = p2_3.split(", ");
            qz3 = Double.parseDouble(p2_3_con[1]);
        }
        while (!p2_3_con[0].equals("qua_z") && loopCounter < LOOPSIZE){
            moveTo(target2_3_v.add(new Vec3(0, 0, inc)), target2_3_q);
            p2_3 = scanOnecBarcode(QRInfoType.QuaZ);
            if (!p2_3.equals("error")) {
                p2_3_con = p2_3.split(", ");
                qz3 = Double.parseDouble(p2_3_con[1]);
            }
            loopCounter++;
        }
        loopCounter = 0;
        Log.d(LOGTAG, "p2_3 = " + p2_3);

        WrapQuaternion yminqua = new WrapQuaternion(0, 0, 0.707f, -0.707f);

        double pw3 = Math.sqrt(1 - (qx3 * qx3) - (qy3 * qy3) - (qz3 * qz3));
        //Result.Status q3status =moveTo(px3,py3,pz3,qx3,qy3,qz3,pw3);
        Result.Status q3status =moveTo(px3,py3,pz3,0, 0, 0.707f, -0.707f);
        WrapQuaternion p3q=new WrapQuaternion((float)qx3,(float)qy3,(float)qz3,(float)pw3);
        Vec3 p3v=WrapQuaternion.vec3mul(p3q,new Vec3(1,0,0)).normalization();
        double ARy=-9.9-py3;
        Vec3 tv= new Vec3(px3,py3,pz3).add(p3v.mul( ARy/p3v.getY()));
        //moveTo(tv.getX(),py3,tv.getZ(),0, 0, 0.707f, -0.707f);
        Log.d(LOGTAG, "p3v"+p3v.toString()+",ARy"+ARy+"tv"+tv.toString());


        if (q3status == null ||q3status != Result.Status.OK){
            Log.d(LOGTAG,"q3status != Result.Status.OK True");
            moveTo(new Vec3(11.45,-8.41,5.43), yminqua);
            moveTo(px3,py3,pz3,qx3,qy3,qz3,pw3);
        }

        Mat ids = new Mat();

        double ar_roll_angle=0, ar_pitch_angle=0, ar_yaw_angle=0, ar_x=0, ar_y=0, ar_z=0;

        Vec3 camerapos = new Vec3();

        // get target pos
        Vec3 targetpos = new Vec3();

        while (arv == 0 && loopCounter < LOOPSIZE) {
            try {
                Log.d(LOGTAG,"Try Read AR! Start");

                Mat source = tryMatNavCam();
                ImageWrite(source, "DICT_5X5_250");

                Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
                List<Mat> corners = new ArrayList<>();
                Log.d(LOGTAG,"Try Read AR! Aruco.detectMarkers start");
                Aruco.detectMarkers(source, dictionary, corners, ids);
                Log.d(LOGTAG,"Try Read AR! Aruco.detectMarkers end");

                if (0 < corners.size()) {
                    Log.d(LOGTAG,"Try Read AR! detected!");
                    arv = (int) ids.get(0, 0)[0];
                    Log.d(LOGTAG,"Try Read AR! arv: "+arv);

                    Mat rotationMatrix = new Mat(), translationVectors = new Mat();
                    Aruco.estimatePoseSingleMarkers(corners, arucoMarkerLength, cameraMatrix, distorsionMatrix,
                            rotationMatrix, translationVectors);
                    Log.d(LOGTAG,"Try Read AR! estimatePoseSingleMarkers!");
                    Log.d(LOGTAG,"Try Read AR! distorsionMatrix: "+translationVectors.dump());
//                    ar_x = translationVectors.get(0,0)[0];
//                    ar_y = translationVectors.get(0, 0)[1];
//                    ar_z = translationVectors.get(0, 0)[2];
                    ar_x = translationVectors.get(0,0)[0];
                    ar_y = translationVectors.get(0, 0)[2];
                    ar_z = translationVectors.get(0, 0)[1];

                    // get target pos
                    robotpos = Vec3PositionNow();
                    robotqt = QuaPositionNow();
                    camerapos = getNavCamVec(robotpos, robotqt);
                    targetpos = targetVec(new Vec3(ar_x, -ar_y, ar_z), camerapos);

                    Log.d(LOGTAG, "targetpos readAR: "+targetpos.toString());
                    Log.d(LOGTAG, "camerapos readAR: "+camerapos.toString());

                    Log.d(LOGTAG,"Try Read AR! rotationMatrix: "+rotationMatrix.dump());
                    ar_roll_angle = rotationMatrix.get(0, 0)[0];//roll
                    ar_pitch_angle = rotationMatrix.get(0, 0)[1];//pitch
                    ar_yaw_angle = rotationMatrix.get(0, 0)[2];//yaw

                    if (ar_pitch_angle < 0) {
                        ar_roll_angle = -ar_roll_angle;
                        ar_pitch_angle = -ar_pitch_angle;
                        ar_yaw_angle = -ar_yaw_angle;
                    }
                    Log.d(LOGTAG,"Try Read AR! ar_x: "+ar_x);
                    Log.d(LOGTAG,"Try Read AR! ar_y: "+ar_y);
                    Log.d(LOGTAG,"Try Read AR! ar_z: "+ar_z);
                    Log.d(LOGTAG,"Try Read AR! ar_roll_angle: "+ar_roll_angle);
                    Log.d(LOGTAG,"Try Read AR! ar_pitch_angle: "+ar_pitch_angle);
                    Log.d(LOGTAG,"Try Read AR! ar_yaw_angle: "+ar_yaw_angle);
                }else{
//                    relativeMoveTo(new Vec3(0,0,0.1),yminqua);
                    Log.d(LOGTAG,"Try Read AR! notfound ar");

                }
            } catch (Exception e) {
                Log.d(LOGTAG, "exception readAR"+ e.getLocalizedMessage());
            }
            loopCounter++;
        }

        if (arv != 0){
            api.judgeSendDiscoveredAR(Integer.toString(arv));
            relativeMoveTo(new Vec3(0,0,0), yminqua);
//            double xzsize = arucoToTargetDist/Math.sqrt(2);
//            Vec3 calmovetopoint = new Vec3(0+(xzsize+ar_x)-NavLaserGap[0],0,0+(xzsize+ar_z)-NavLaserGap[2]);
//            moveTo(calmovetopoint, yminqua);

            // TODO:: new impl laser shoutting
            // get my pos
            robotpos = Vec3PositionNow();
            robotqt = QuaPositionNow();
            Vec3 laserpos = getLaserVec(robotpos, robotqt);
            Log.d(LOGTAG, "laserpos readAR "+laserpos.toString());

//             get angle
            WrapQuaternion ar_q = getTargetRotationMinetaAngle(targetpos, laserpos, robotpos);

            relativeMoveTo(new Vec3(0,0,0), ar_q);
//            moveTo(robotpos, ar_q);

            Log.d(LOGTAG,"DO LASERCTL!");
            api.laserControl(true);
        }else{
            Log.d(LOGTAG, "failed readAR action:(");
        }
        Mat finishimage = tryMatNavCam();
        ImageWrite(finishimage, "FINISH");

        api.judgeSendFinishSimulation();

    }

    @Override
    protected void runPlan3() {
        // write here your plan 3

        Log.i(LOGTAG, "apijudgeSendStart");
        api.judgeSendStart();
        //TODO: もっと的に近付ける
        double distance = 0.12;
        Vec3 road1_1 = new Vec3(11.15, -4.8, 4.55);
        Vec3 target1_1 = new Vec3(11.5 - distance, -5.7, 4.5);
        Vec3 target1_2 = new Vec3(11, -6, 5.55 - distance);
        Vec3 target1_3 = new Vec3(11, -5.5, 4.33 + distance);

        moveTo(road1_1, new Quaternion(0, 0, 0.707f, -0.707f));
        moveTo(target1_3, new Quaternion(0, 0.707f, 0, 0.707f));
        moveTo(target1_1, new Quaternion(0, 0, 0, -1));
        moveTo(target1_2, new Quaternion(0, -0.707f, 0, 0.707f));

        Vec3 road2_1 = new Vec3(10.5, -6.45, 5.1);
        Vec3 road2_2 = new Vec3(11.35, -7.3, 4.9);
        Vec3 target2_1 = new Vec3(10.30 + distance, -7.5, 4.7);
        Vec3 target2_2 = new Vec3(11.5 - distance, -8, 5);
        Vec3 target2_3 = new Vec3(11, -7.7, 5.55 - distance);

        moveTo(road2_1,new Quaternion(0, 0, 0.707f, -0.707f));
        moveTo(road2_2,new Quaternion(0, 0, 0.707f, -0.707f));
        moveTo(target2_1,new Quaternion(0, 0, 1,0));
        moveTo(target2_2,new Quaternion(0, 0, 0,-1));
        moveTo(target2_3,new Quaternion(0, -0.707f, 0, 0.707f));

        Vec3 road3_1 = new Vec3(11.45,-8.4,5.3);
        Vec3 road3_2=new Vec3(11.95,-9.2,5.3);
        moveTo(road3_1,new Quaternion(0, 0, 0.707f, -0.707f));
        moveTo(road3_2,new Quaternion(0, 0, 0.707f, -0.707f));

        api.laserControl(true);
        api.judgeSendFinishSimulation();
    }

    //-------------ctl functions--------------------
    private Result.Status keepInZone(){
        Log.i(LOGTAG, "keepInZone fixer start");

        final double[] kiz_x = {10.25,11.65};
        final double[] kiz_y = {-9.75,-3};
        final double[] kiz_z= {4.2,5.6};

        double xx = 0;
        double yy = 0;
        double zz = 0;

        double x =  api.getTrustedRobotKinematics().getPosition().getX();
        double y =  api.getTrustedRobotKinematics().getPosition().getY();
        double z =  api.getTrustedRobotKinematics().getPosition().getZ();
        double qx =  api.getTrustedRobotKinematics().getOrientation().getX();
        double qy =  api.getTrustedRobotKinematics().getOrientation().getY();
        double qz =  api.getTrustedRobotKinematics().getOrientation().getZ();
        double qw =  api.getTrustedRobotKinematics().getOrientation().getZ();

        Log.d(LOGTAG,"Keep In Zone");
        if (x < kiz_x[0]){
            xx = kiz_x[0] - x + 0.1;
        }
        else if (x > kiz_x[1]){
            xx = x - kiz_x[0] - 0.1;
        }
        if (y < kiz_y[0]){
            yy = kiz_y[0] - y + 0.1;
        }
        else if (y > kiz_y[1]){
            yy = y - kiz_y[0] - 0.1;
        }
        if (z < kiz_z[0]){
            zz = kiz_z[0] - z + 0.1;
        }
        else if (z > kiz_z[1]){
            zz = z - kiz_z[0] - 0.1;
        }
        Log.i(LOGTAG, "keepInZone fixer xx:" + xx);
        Log.i(LOGTAG, "keepInZone fixer yy:" + yy);
        Log.i(LOGTAG, "keepInZone fixer zz:" + zz);
        if (xx !=0 || yy != 0|| zz!=0){
            return relativeMoveTo(new Vec3(xx, yy, zz), new WrapQuaternion((float) qx,(float) qy,(float)qz,(float)qw));
        }
        return null;
    }

    private String scanOnecBarcode(QRInfoType type){
        Log.d(LOGTAG,"start scanBarcode");
        Mat snapshot = api.getMatNavCam();
        String value = detectQrcode(snapshot, type.name());
        if (!value.equals("error") && value.split(", ")[0].equals(type.getKey())) {
            api.judgeSendDiscoveredQR(type.getInt() , value);
            Log.d(LOGTAG,"valuesQR" + value);
        }
        else{
            Log.d(LOGTAG,"valuesQR = error");
        }
        return value;
    }

    private String scanTryBarcode(Vec3 vec, WrapQuaternion qua, QRInfoType type){
        Log.d(LOGTAG,"start scanTryBarcode");
        final int LOOP_MAX = 3;
        int loopCounter = 0;
        String value = "";

        while (loopCounter < LOOP_MAX || value == "error") {
            Mat snapshot = api.getMatNavCam();
            value = detectQrcode(snapshot, type.name());
            if (!value.equals("error") && value.split(", ")[0].equals(type.getKey())) {
                api.judgeSendDiscoveredQR(type.getInt() , value);
                Log.d(LOGTAG,"valuesQR" + value);
            }
            else{
                Log.d(LOGTAG,"valuesQR = error");
            }
            moveTo(vec, qua);
            loopCounter++;
        }
        return value;
    }

    private String scanKinematicBarcode(Vec3 vec, WrapQuaternion qua, QRInfoType type){
        return scanKinematicBarcode(vec.getX(), vec.getY(), vec.getZ(),
                (double)qua.getX(), (double)qua.getY(), (double)qua.getZ(), (double)qua.getW(),
                type
        );
    }

    private String scanKinematicBarcode(double pos_x, double pos_y, double pos_z,
                                double qua_x, double qua_y, double qua_z,
                                double qua_w,QRInfoType type){
        Vec3 updatePoint = new Vec3(0,0,0);
        WrapQuaternion updateQuaternion = new WrapQuaternion();
        int loopMax = 3;
        int loop = 0;
        Mat snapshot = tryMatNavCam();
        String value = detectQrcode(snapshot, type.name());
        printAllPosition("position Scan");
        while (value.equals("error") && loop < loopMax) {
            moveTo(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
            snapshot = tryMatNavCam();
            value = detectQrcode(snapshot, type.name());
            if (!value.equals("error") && value.split(", ")[0].equals(type.getKey())) {
                double qx = api.getTrustedRobotKinematics().getOrientation().getX();
                double qy = api.getTrustedRobotKinematics().getOrientation().getY();
                double qz = api.getTrustedRobotKinematics().getOrientation().getZ();
                double qw = api.getTrustedRobotKinematics().getOrientation().getW();
                printAllPosition("---before sep---");
                updateQuaternion = relativeStableQuaternion(
                        new WrapQuaternion((float) qua_x, (float) qua_y, (float) qua_z, (float) qua_w),
                        new WrapQuaternion((float) qx, (float) qy, (float) qz, (float) qw)
                );

                Log.d(LOGTAG, "relative Quatation");
                relativeMoveTo(updatePoint, updateQuaternion);
                snapshot = tryMatNavCam();
                value = detectQrcode(snapshot, type.name());

                printPosition("relative last", updatePoint, updateQuaternion);
            }
            loop++;
        }
        if (!value.equals("error") && value.split(", ")[0].equals(type.getKey())) {
            api.judgeSendDiscoveredQR(type.getInt() , value);
            System.out.println("valuesQR" + value);
        }
        else{
            System.out.println("valuesQR = null");
        }
        return value;
    }
    private String scanBarcodeMoveTo(Vec3 vec, WrapQuaternion qua, QRInfoType type){
        return scanBarcodeMoveTo(vec.getX(), vec.getY(), vec.getZ(),
                (double)qua.getX(), (double)qua.getY(), (double)qua.getZ(), (double)qua.getW(),
                type
        );
    }

    private String scanBarcodeMoveTo(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w, QRInfoType type){
        Log.d(LOGTAG,"start scanBarcodeMoveTo");
        int loopMax = 10;
        int loop = 0;
        moveTo(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
        Mat snapshot = tryMatNavCam();
        String value = detectQrcode(snapshot, type.name());
        double viewP = 0.025;
        while (value.equals("error") && loop < loopMax) {
            if (loop%8 == 0) moveTo(pos_x,pos_y,pos_z,qua_x + viewP,qua_y,qua_z,qua_w);
            else if (loop%8 == 1) moveTo(pos_x,pos_y,pos_z,qua_x,qua_y + viewP,qua_z,qua_w);
            else if (loop%8 == 2) moveTo(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z + viewP,qua_w);
            else if (loop%8 == 3) moveTo(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w + viewP);
            else if (loop%8 == 4) moveTo(pos_x,pos_y,pos_z,qua_x + viewP,qua_y + viewP,qua_z,qua_w);
            else if (loop%8 == 5) moveTo(pos_x,pos_y,pos_z,qua_x,qua_y + viewP,qua_z + viewP,qua_w);
            else if (loop%8 == 6) moveTo(pos_x,pos_y,pos_z,qua_x + viewP,qua_y,qua_z + viewP,qua_w);
            else if (loop%8 == 7) moveTo(pos_x,pos_y,pos_z,qua_x + viewP,qua_y + viewP,qua_z + viewP,qua_w);
            else  moveTo(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w);
            snapshot = tryMatNavCam();
            value = detectQrcode(snapshot, type.name());
            loop++;
        }
        if (!value.equals("error") && value.split(", ")[0].equals(type.getKey())) {
            api.judgeSendDiscoveredQR(type.getInt() , value);
            Log.d(LOGTAG,"valuesQR" + value);
        }
        else{
            Log.d(LOGTAG,"valuesQR = error");
        }
        return value;
    }

    private String scanBarcodeRelation(WrapQuaternion qua, QRInfoType type){
        return scanBarcodeRelation((double)qua.getX(), (double)qua.getY(), (double)qua.getZ(), (double)qua.getW(), type);
    }

    private String scanBarcodeRelation(double qua_x, double qua_y, double qua_z, double qua_w, QRInfoType type){
        Log.d(LOGTAG,"start scanBarcodeRelation");
        int loopMax = 10;
        int loop = 0;
        Mat snapshot = tryMatNavCam();
        String value = detectQrcode(snapshot, type.name());
        double viewP = 0.025;
        while (value == "error" && loop < loopMax) {
            if (loop%8 == 0) relativeMoveTo(0,0,0,qua_x + viewP,qua_y,qua_z,qua_w);
            else if (loop%8 == 1) relativeMoveTo(0,0,0,qua_x,qua_y + viewP,qua_z,qua_w);
            else if (loop%8 == 2) relativeMoveTo(0,0,0,qua_x,qua_y,qua_z + viewP,qua_w);
            else if (loop%8 == 3) relativeMoveTo(0,0,0,qua_x,qua_y,qua_z,qua_w + viewP);
            else if (loop%8 == 4) relativeMoveTo(0,0,0,qua_x + viewP,qua_y + viewP,qua_z,qua_w);
            else if (loop%8 == 5) relativeMoveTo(0,0,0,qua_x,qua_y + viewP,qua_z + viewP,qua_w);
            else if (loop%8 == 6) relativeMoveTo(0,0,0,qua_x + viewP,qua_y,qua_z + viewP,qua_w);
            else if (loop%8 == 7) relativeMoveTo(0,0,0,qua_x + viewP,qua_y + viewP,qua_z + viewP,qua_w);
            else  relativeMoveTo(0,0,0,qua_x,qua_y,qua_z,qua_w);
            snapshot = tryMatNavCam();
            value =  detectQrcode(snapshot, type.name());
            loop++;
        }
        if (!value.equals("error") && value.split(", ")[0].equals(type.getKey())) {
            api.judgeSendDiscoveredQR(type.getInt() , value);
            Log.d(LOGTAG,"valuesQR" + value);
        }
        else{
            Log.d(LOGTAG,"valuesQR = error");
        }
        return value;
    }

    //-------------QR Decoding--------------------

    private Boolean ImageWrite(Mat mat, String key){
        if (!DOJAXA) {
            if (mat == null) {
                return false;
            }
            Bitmap bitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(mat, bitmap);
            String uniqueString = UUID.randomUUID().toString();
            try {
                File path = getExternalFilesDir(Environment.DIRECTORY_PICTURES);
                File file = new File(path, EPOCK_UNIQUE_STR + "-" + uniqueString);
                FileOutputStream out = new FileOutputStream(file + "-" + key + ".png");
                Boolean ok = bitmap.compress(Bitmap.CompressFormat.PNG, 90, out);
                Log.d(LOGTAG, "ImageWrite: " + file + "-" + key + ".png");
                Log.d(LOGTAG, "ImageWrite status: " + ok);
                out.close();

                return ok;
            } catch (Exception e) {
                Log.d(LOGTAG, "ImageWrite fail: " + e.getLocalizedMessage());
            }
        }
        return false;
    }

    // If more than one QRcode finder pattern is found, the value is set to True.
    // cf, https://github.com/opencv/opencv/blob/c3e8a82c9c0b18432627fe100db332b42d64b8d3/modules/objdetect/src/qrcode.cpp#L137
    private Boolean isQRFinderPatternChecking(Mat nmat){
        Log.d(LOGTAG,"isQRFinderPatternChecking start");

        if(nmat == null || nmat.empty()){
            Log.d(LOGTAG,"nmat == null");
            return false;
        }
        Imgproc.threshold(nmat, nmat, 0, 255, Imgproc.THRESH_BINARY|Imgproc.THRESH_OTSU);

        final int height_bin_barcode = nmat.rows();
        final int width_bin_barcode = nmat.cols();
        final int test_lines_size = 5;
        double[] test_lines = new double[test_lines_size];
        List<Integer> pixels_position = new ArrayList<Integer>();
        final double eps_vertical = 0.2;

        //Only half of the image is used for fast judgment.
        for (int y=0; y < height_bin_barcode/2; y++){
            pixels_position.clear();

            int pos = 0;
            for(; pos < width_bin_barcode; pos++){
                if (nmat.get(y, pos)[0] == 0){break;}
            }
            if (pos == width_bin_barcode) { continue; }

            pixels_position.add(pos);
            pixels_position.add(pos);
            pixels_position.add(pos);

            int future_pixel = 255;

            for(int x=pos; x<width_bin_barcode; x++){
                if (nmat.get(y, x)[0] == future_pixel){
                    future_pixel = ~future_pixel;
                    pixels_position.add(x);
                }
            }

            pixels_position.add(width_bin_barcode -1);
            for (int i=2; i<pixels_position.size() -4; i+=2){
                test_lines[0] = (double)(pixels_position.get(i-1) - pixels_position.get(i-2));
                test_lines[1] = (double)(pixels_position.get(i  ) - pixels_position.get(i-1));
                test_lines[2] = (double)(pixels_position.get(i+1) - pixels_position.get(i  ));
                test_lines[3] = (double)(pixels_position.get(i+2) - pixels_position.get(i+1));
                test_lines[4] = (double)(pixels_position.get(i+3) - pixels_position.get(i+2));

                double length = 0.0, weight = 0.0;

                for (int j=0; j<test_lines_size; j++){ length += test_lines[j];}

                if (length == 0){continue;}
                for (int j=0; j<test_lines_size; j++){
                    if (j != 2) { weight += Math.abs((test_lines[j] / length) - 1.0/7.0); }
                    else        { weight += Math.abs((test_lines[j] / length) - 3.0/7.0); }
                }
                Log.d(LOGTAG,"isQRFinderPatternChecking test_lines"+ test_lines);
                Log.d(LOGTAG,"isQRFinderPatternChecking weight"+ weight);

                if (weight < eps_vertical){
                    return true;
                }
            }
        }
        Log.d(LOGTAG,"isQRFinderPatternChecking not found");

        return false;
    }

    private String detectQrcode(Mat nmat, String key) {
        Log.d(LOGTAG,"start detectQrcode");
        if(nmat == null || nmat.empty()){
            Log.d(LOGTAG,"nmat == null");
            return "error";
        }

        ImageWrite(nmat, key);

        //1: qr rect base cutimage & decode
        List<Mat> points = QRRectTrimPoint(nmat);
        String result = "";
        if(points != null){
            Log.d(LOGTAG,"detectQrcode 1: points.size(): " + points.size());
            for(int i=0; i< points.size(); i++){
                Mat writemat = pointCuting(nmat, points.get(i));
                ImageWrite(writemat, key+"-pointCuting");

//                Boolean isqrfinder = isQRFinderPatternChecking(writemat);
//                Log.d(LOGTAG,"detectQrcode 1: isQRFinderPatternChecking(writemat): " +isqrfinder);
//                if (!isqrfinder){
//                    continue;
//                }

//                writemat = sharpenFilter(writemat);
//                ImageWrite(writemat, key+"-sharpenFilter");
                try {
                    result = zxingDetectDecodeQrcode(writemat);
                    if (!result.equals("error") && 0 < result.length()){
                        return result;
                    }
                }catch (Exception e){
                    Log.d(LOGTAG, "1: qr rect base cutimage & decode/exception detectQrcode: "+ e.getLocalizedMessage());
                }
            }
        }


        Log.d(LOGTAG,"detectQrcode not working all pattern:(");
        return "error";
    }

    private String zxingDetectDecodeQrcode(Mat nmat){
        if(nmat == null){
            Log.d(LOGTAG,"nmat == null");
            return "error";
        }

        Bitmap bitmap = Bitmap.createBitmap(nmat.width(), nmat.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(nmat, bitmap);
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();

        int[] pixels = new int[width * height];
        bitmap.getPixels(pixels, 0, width, 0, 0, width, height);
        Log.d(LOGTAG, "zxingDetectQRcode try");

        try {
            // zxing で扱える BinaryBitmap形式に変換する
            LuminanceSource source = new RGBLuminanceSource(width, height, pixels);
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
            Log.d(LOGTAG, "zxingDetectQRcode conv bitmap");

            // zxing で画像データを読み込み解析する
            Reader reader = new QRCodeReader();

            // do optimize. not speedy
            Map<DecodeHintType, Object> hintMap = new HashMap<DecodeHintType, Object>();
            hintMap.put(DecodeHintType.TRY_HARDER, Boolean.TRUE);
            com.google.astrozxing.Result decodeResult = reader.decode(binaryBitmap, hintMap);

//            com.google.astrozxing.Result decodeResult = reader.decode(binaryBitmap);

            // 解析結果を取得する
            String result = decodeResult.getText();
            if(!(0 < result.length()) || result == null) {
                return "error";
            }
            Log.d(LOGTAG,"readQR"+result);
            return result;
        } catch (Exception e) {
            Log.d(LOGTAG, "exception readQR"+ e.getLocalizedMessage());
        }
        return "error";
    }

    private Mat pointCuting(Mat nmat, Mat point){
        Log.d(LOGTAG,"pointCuting try");
        Log.d(LOGTAG,"pointCuting point channels" + point.channels());
        Log.d(LOGTAG,"pointCuting point depth" + point.depth());
        Log.d(LOGTAG,"pointCuting point cols" + point.cols());
        Log.d(LOGTAG,"pointCuting point rows" + point.rows());
        Log.d(LOGTAG,"pointCuting point type" + point.type());
        Log.d(LOGTAG,"pointCuting point.type()" + CvType.typeToString(point.type()));
        Log.d(LOGTAG,"pointCuting point.dump()" + point.dump());

        Log.d(LOGTAG,"pointCuting nmat channels" + nmat.channels());
        Log.d(LOGTAG,"pointCuting nmat depth" + nmat.depth());
        Log.d(LOGTAG,"pointCuting nmat cols" + nmat.cols());
        Log.d(LOGTAG,"pointCuting nmat rows" + nmat.rows());
        Log.d(LOGTAG,"pointCuting nmat type" + nmat.type());
        Log.d(LOGTAG,"pointCuting nmat.type()" + CvType.typeToString(nmat.type()));


        float distPoint[] = new float[]{0,400, 0,0 ,400,0 ,400,400};//Lower left, upper left, upper right, lower left.
        Mat dstmat = new Mat(4,2, CvType.CV_32F);
        dstmat.put(0,0, distPoint);
        Log.d(LOGTAG,"pointCuting dstmat channels" + dstmat.channels());
        Log.d(LOGTAG,"pointCuting dstmat depth" + dstmat.depth());
        Log.d(LOGTAG,"pointCuting dstmat cols" + dstmat.cols());
        Log.d(LOGTAG,"pointCuting dstmat rows" + dstmat.rows());
        Log.d(LOGTAG,"pointCuting dstmat.type()" + CvType.typeToString(dstmat.type()));
        Log.d(LOGTAG,"pointCuting dstmat.dump()" + dstmat.dump());

        Mat rmat = Imgproc.getPerspectiveTransform(point, dstmat);
        Log.d(LOGTAG,"getPerspectiveTransform done!!!");
        Log.d(LOGTAG,"pointCuting rmat channels" + rmat.channels());
        Log.d(LOGTAG,"pointCuting rmat depth" + rmat.depth());
        Log.d(LOGTAG,"pointCuting rmat cols" + rmat.cols());
        Log.d(LOGTAG,"pointCuting rmat rows" + rmat.rows());
        Log.d(LOGTAG,"pointCuting rmat.type()" + CvType.typeToString(rmat.type()));
        Log.d(LOGTAG,"pointCuting rmat.dump()" + rmat.dump());

        Mat mat = new Mat();
        Imgproc.warpPerspective(nmat, mat, rmat, new Size(400,400));

        return mat;
    }

    private List<Mat> QRRectTrimPoint(Mat nmat){
        Log.d(LOGTAG,"QRRectTrimPoint try");

        Mat fmap = nmat.clone();

        Imgproc.threshold(fmap, fmap, 200, 255, Imgproc.THRESH_BINARY_INV);
        ImageWrite(fmap, "threshold1");

        Core.bitwise_not(fmap, fmap);
        ImageWrite(fmap, "bitwise_not");

        Mat hierarchy = Mat.zeros(new Size(5,5), CvType.CV_8UC1);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(fmap, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Log.d(LOGTAG,"QRRectTrimPoint contours.size: " + contours.size());

        List<Mat> retcontour = new ArrayList<Mat>();

        if (contours.size() <= 0){
            return null;
        }

        for(int i = 0; i<contours.size(); i++){
            MatOfPoint ptmat = contours.get(i);
            double size = Imgproc.contourArea(ptmat);

            if (size < nmat.size().area() / (100 * 1)) {
                /* Ignore areas that are small in size. */
                continue;
            }

            MatOfPoint2f contours2f = new MatOfPoint2f(ptmat.toArray());
            MatOfPoint2f approx2f = new MatOfPoint2f();
            double ep = Imgproc.arcLength(contours2f, true);
            Imgproc.approxPolyDP(contours2f, approx2f, ep * 0.05, true);

            // Obtaining a convex hull
            MatOfPoint approx = new MatOfPoint(approx2f.toArray());
            MatOfInt hull = new MatOfInt();
            Imgproc.convexHull(approx, hull);


            if(hull.size().height == 4){
                Mat srcPointMat = new Mat(4,2,CvType.CV_32F);

                double[] md = approx.get((int)hull.get(1,0)[0], 0);
                srcPointMat.put(1, 0, new float[]{(float)md[0], (float)md[1]});

                md = approx.get((int)hull.get(2,0)[0], 0);
                srcPointMat.put(2, 0, new float[]{(float)md[0], (float)md[1]});

                md = approx.get((int)hull.get(3,0)[0], 0);
                srcPointMat.put(3, 0, new float[]{(float)md[0], (float)md[1]});

                md = approx.get((int)hull.get(0,0)[0], 0);
                srcPointMat.put(0, 0, new float[]{(float)md[0], (float)md[1]});
                retcontour.add(srcPointMat);
            }
        }
        return retcontour;
    }

    private Mat sharpenFilter(Mat nmat){
        Log.d(LOGTAG,"sharpenFilter try!");

        final float k = -1;
        Mat kernel = new Mat(3, 3, CvType.CV_32FC1);
        kernel.put(0, 0, new float[]{k,k,k});
        kernel.put(1, 0, new float[]{k,9,k});
        kernel.put(2, 0, new float[]{k,k,k});
        Log.d(LOGTAG,"sharpenFilter write kernel!");

        Mat destination=new Mat(nmat.rows(),nmat.cols(), CvType.CV_8UC1);
        Imgproc.filter2D(nmat, destination, destination.depth(), kernel);
        Log.d(LOGTAG,"Imgproc.filter2D done!");

        return destination;
    }

    private Mat normalBinaryFilter1(Mat nmat){
        Log.d(LOGTAG,"normalBinaryImage try!");

        Imgproc.threshold(nmat, nmat, 200, 255, Imgproc.THRESH_BINARY_INV);
        ImageWrite(nmat, "normalBinaryImage threshold1");

        Core.bitwise_not(nmat, nmat);
        ImageWrite(nmat, "normalBinaryImage bitwise_not");

        return nmat;
    }

    private Mat normalBinaryFilter2(Mat nmat){
        Log.d(LOGTAG,"normalBinaryFilter2 try!");

        Core.bitwise_not(nmat, nmat);
        ImageWrite(nmat, "normalBinaryImage bitwise_not");

        Imgproc.threshold(nmat, nmat, 200, 255, Imgproc.THRESH_BINARY_INV);
        ImageWrite(nmat, "normalBinaryImage threshold1");

        return nmat;
    }
    //-----------------AR processing----------------------

    private WrapQuaternion getTargetRotationMinetaAngle(Vec3 target, Vec3 laser, Vec3 center){
        Log.d(LOGTAG, "getTargetRotationMinetaAngle start!");

        double X = target.getX()- center.getX();
        double Y = center.getY()- target.getY();
        double XY = Math.sqrt(X*X + Y*Y);

        double ztheta = 0;
        double theta11 = Math.toDegrees(Math.atan(X/Y));
        double theta12 = 0;

        if (laser.getX() < center.getX()){
            theta12 =  Math.toDegrees(Math.asin((center.getX() - laser.getX())/XY));
            ztheta = theta11 + theta12;
        }else if (laser.getX() > center.getX()){
            theta12 =  Math.toDegrees(Math.asin((laser.getX() - center.getX())/XY));
            ztheta = theta11 - theta12;
        }else{
            ztheta = theta11;
        }

        double Z = center.getZ() - target.getZ();
        double YZ = Math.sqrt(Y*Y + Z*Z);

        double xtheta = 0;
        double theta21 =  Math.toDegrees(Math.atan(Z/Y));
        double theta22 = 0;

        if (laser.getZ() < center.getZ()){
            theta22 =  Math.toDegrees(Math.asin((center.getZ() - laser.getZ())/YZ));
            xtheta = theta21 - theta22;
        }else if (laser.getX() > center.getX()){
            theta22 =  Math.toDegrees(Math.asin((laser.getZ() - center.getZ())/YZ));
            xtheta = theta21 + theta22;
        }else{
            xtheta = theta21;
        }

        Vec3 theta = new Vec3(xtheta, 0, ztheta-90);
        Log.d(LOGTAG, "getTargetRotationAngle theta: "+ theta.toString());


        return WrapQuaternion.EulerZYX(theta);
    }

    private WrapQuaternion getTargetRotationAngle(Vec3 target, Vec3 laser, Vec3 center){
        Log.d(LOGTAG, "getTargetRotationAngle start!");

        Vec3 TOvec = target.sub(center);
        Log.d(LOGTAG, "getTargetRotationAngle TOvec: "+ TOvec.toString());

        Vec3 LTvec = laser.sub(center);
        Log.d(LOGTAG, "getTargetRotationAngle LTvec: "+ LTvec.toString());

        //XY
        double ztheta = Math.toDegrees(Math.acos(Vec3.dot2vecNormal(TOvec.getX(),TOvec.getY(),LTvec.getX(),LTvec.getY())));
        //XZ
        double ytheta = Math.toDegrees(Math.acos(Vec3.dot2vecNormal(TOvec.getX(),TOvec.getZ(),LTvec.getX(),LTvec.getZ())));
        //ZY
        double xtheta = Math.toDegrees(Math.acos(Vec3.dot2vecNormal(TOvec.getZ(),TOvec.getY(),LTvec.getZ(),LTvec.getY())));

        Vec3 theta = new Vec3(xtheta, ytheta, ztheta);
        Log.d(LOGTAG, "getTargetRotationAngle theta: "+ theta.toString());


        return WrapQuaternion.EulerZYX(theta);
    }

    private Vec3 targetVec(Vec3 arvec, Vec3 center){
        double xzsize = arucoToTargetDist/Math.sqrt(2);

        return arvec.add(new Vec3(xzsize, 0, xzsize)).add(center);
    }
    //-----------------basic action functions----------------------

    private WrapQuaternion relativeStableQuaternion(WrapQuaternion targetQua, WrapQuaternion currentQua){
        printPosition(
                "relative stable target",
                0,0,0,(double) targetQua.getX()
                ,(double) targetQua.getY(),(double) targetQua.getZ(),(double) targetQua.getW());
        printPosition(
                "relative stable current",
                0,0,0,(double) currentQua.getX()
                ,(double) currentQua.getY(),(double) currentQua.getZ(),(double) currentQua.getW());

        WrapQuaternion inverse = WrapQuaternion.Inverse(currentQua);
        WrapQuaternion mul = WrapQuaternion.mul(inverse,targetQua);

        printPosition(
                "relative stable result",
                0,0,0,
                (double) mul.getX(),(double) mul.getY(),(double) mul.getZ(),(double) mul.getW());

        return mul;
    }

    public Result.Status moveTo(Vec3 vec, WrapQuaternion quat) {
        return moveToRun(
                vec.getX(), vec.getY(), vec.getZ(),
                quat.getX(),quat.getY(), quat.getZ(), quat.getW()
        );
    }
    public Result.Status moveTo(double pos_x, double pos_y, double pos_z,
                       double qua_x, double qua_y, double qua_z, double qua_w) {
        return moveToRun(
                pos_x, pos_y, pos_z,
                (float) qua_x, (float)qua_y, (float)qua_z, (float)qua_w
        );
    }
    private Result.Status moveToRun(double pos_x, double pos_y, double pos_z,
                           float qua_x, float qua_y, float qua_z, float qua_w) {

        final int LOOP_MAX = 4;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion(qua_x, qua_y, qua_z, qua_w);

        Result result = this.api.moveTo(point, quaternion, true);
        int loopCounter = 0;
        Log.i(MainService.LOGTAG, "MoveTo Result: " + result.getStatus().toString());
        printPosition("moveToRun", point, quaternion);

        while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
            result = this.api.moveTo(point, quaternion, true);
            Log.i(MainService.LOGTAG, "MoveTo Result: " + result.getStatus().toString());
            printPosition("moveToRun", point, quaternion);
            ++loopCounter;
        }
        if(result.getStatus() == null ||result.getStatus() == Result.Status.EXEC_FAILED){
            keepInZone();
        }
        return result.getStatus();
    }

    public Result.Status relativeMoveTo(Vec3 vec, WrapQuaternion quat) {
        return relativeMoveToRun(
                vec.getX(), vec.getY(), vec.getZ(),
                quat.getX(),quat.getY(), quat.getZ(), quat.getW()
        );
    }
    public Result.Status relativeMoveTo(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z, double qua_w) {
        return relativeMoveToRun(
                pos_x, pos_y, pos_z,
                (float) qua_x, (float)qua_y, (float)qua_z, (float)qua_w
        );
    }
    private Result.Status relativeMoveToRun(double pos_x, double pos_y, double pos_z,
                                   float qua_x, float qua_y, float qua_z, float qua_w){
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion(qua_x, qua_y, qua_z, qua_w);

        final int LOOP_MAX = 4;
        int loopCounter = 0;
        Result result = api.relativeMoveTo(point, quaternion, true);
        Log.i(MainService.LOGTAG, "relativeMoveTo Result: " + result.getStatus().toString());
        printPosition("relativeMoveTo", point, quaternion);
        while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
            result = this.api.relativeMoveTo(point, quaternion, true);
            Log.i(MainService.LOGTAG, "relativeMoveTo Result: " + result.getStatus().toString());
            printPosition("relativeMoveTo", point, quaternion);

            ++loopCounter;
        }
        return result.getStatus();
    }
    public void moveTo2(double pos_x, double pos_y, double pos_z,
                       double qua_x, double qua_y, double qua_z,
                       double qua_w) {

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = this.api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            result = this.api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    public void moveTo(Point pos, Quaternion qua) {
        this.moveTo2(pos.getX(), pos.getY(), pos.getZ(), qua.getX(), qua.getY(), qua.getZ(), qua.getW());
    }

    private Mat tryMatNavCam() {
        Log.i(MainService.LOGTAG, "tryMatNavCam start");
        final int LOOP_MAX = 3;

        Mat result = this.api.getMatNavCam();
        int loopCounter = 0;
        Log.d(MainService.LOGTAG, "tryMatNavCam Result: " + (result!=null));

        while (result == null && loopCounter < LOOP_MAX) {
            result = this.api.getMatNavCam();
            Log.d(MainService.LOGTAG, "tryMatNavCam Result: " + (result!=null));
            ++loopCounter;
        }

        return result;
    }

    //-----------------basic utils functions----------------------

    private void printPosition(String label, Point pos, Quaternion qua){
        printPosition(label, pos.getX(), pos.getY(), pos.getZ(), qua.getX(), qua.getY(), qua.getZ(), qua.getW());
    }

    private void printPosition(String label,double x,double y,double z,
                               double qx,double qy, double qz, double qw){
        Log.d(LOGTAG,label + " : Position = " + x + " "
                + y + " " + z + " " + "; Quan " + qx + " " + qy + " " + qz + " " + qw);
    }

    private void printAllPosition(String lable){
        Log.d(LOGTAG,lable);
        double x =  api.getTrustedRobotKinematics().getPosition().getX();
        double y =  api.getTrustedRobotKinematics().getPosition().getY();
        double z =  api.getTrustedRobotKinematics().getPosition().getZ();
        double qx = api.getTrustedRobotKinematics().getOrientation().getX();
        double qy = api.getTrustedRobotKinematics().getOrientation().getY();
        double qz = api.getTrustedRobotKinematics().getOrientation().getZ();
        double qw = api.getTrustedRobotKinematics().getOrientation().getW();
        Log.d(LOGTAG,"IMU linear: " + api.getImu().getLinearAcceleration().toString());
        Log.d(LOGTAG,"IMU Angular: " + api.getImu().getAngularVelocity().toString());
        Log.d(LOGTAG,"IMU Orientation: " + api.getImu().getOrientation().toString());
        printPosition("moving",x,y,z, qx,qy,qz,qw);
        Log.d(LOGTAG,"<" + lable + ">");
    }

    private Vec3 Vec3PositionNow(){
        double x =  api.getTrustedRobotKinematics().getPosition().getX();
        double y =  api.getTrustedRobotKinematics().getPosition().getY();
        double z =  api.getTrustedRobotKinematics().getPosition().getZ();

        return new Vec3(x,y,z);
    }

    private WrapQuaternion QuaPositionNow(){
        double qx = api.getTrustedRobotKinematics().getOrientation().getX();
        double qy = api.getTrustedRobotKinematics().getOrientation().getY();
        double qz = api.getTrustedRobotKinematics().getOrientation().getZ();
        double qw = api.getTrustedRobotKinematics().getOrientation().getW();

        return new WrapQuaternion((float) qx, (float)qy, (float)qz, (float)qw);
    }

    private Vec3 AstrobeeLaserNormalVec(Vec3 vpos, WrapQuaternion qpos){
        Vec3 nav = getNavCamVec(vpos, qpos);
        Vec3 haz = getHazCamVec(vpos, qpos);
        Vec3 laser = getLaserVec(vpos, qpos);

        Vec3 s1 = nav.sub(laser);
        Vec3 s2 = haz.sub(laser);

        return s1.cross(s2);
    }

    private Vec3 getNavCamVec(Vec3 vpos, WrapQuaternion qpos){
        Vec3 v = new Vec3(NavCamvec[0], NavCamvec[1], NavCamvec[2]);
        Vec3 nv = WrapQuaternion.vec3mul(qpos, v);

        return vpos.add(nv);
    }

    private Vec3 getHazCamVec(Vec3 vpos, WrapQuaternion qpos){
        Vec3 v = new Vec3(HazCamvec[0], HazCamvec[1], HazCamvec[2]);
        Vec3 nv = WrapQuaternion.vec3mul(qpos, v);

        return vpos.add(nv);
    }

    private Vec3 getLaserVec(Vec3 vpos, WrapQuaternion qpos){
        Vec3 v = new Vec3(Laservec[0], Laservec[1], Laservec[2]);
        Vec3 nv = WrapQuaternion.vec3mul(qpos, v);

        return vpos.add(nv);
    }
}

