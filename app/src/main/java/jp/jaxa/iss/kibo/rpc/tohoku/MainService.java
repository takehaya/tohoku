package jp.jaxa.iss.kibo.rpc.tohoku;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;

import java.io.File;
import java.io.FileOutputStream;
import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

//import boofcv.abst.fiducial.QrCodeDetector;
//import boofcv.alg.fiducial.qrcode.QrCode;
//import boofcv.android.ConvertBitmap;
//import boofcv.factory.fiducial.FactoryFiducial;
//import boofcv.struct.image.FactoryImage;
//import boofcv.struct.image.GrayU8;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */


public class MainService extends KiboRpcService {
    private List<KiboQRField> QRTable = new ArrayList<KiboQRField>(Arrays.asList(
            new KiboQRField("1-1", 11.5, -5.7, 4.5, 0, 0, 0, 1, QRInfoType.PosX),
            new KiboQRField("1-2", 11, -6, 5.55, 0, -0.7071068, 0, 0.7071068, QRInfoType.PosY),
            new KiboQRField("1-3", 11, -5.5, 4.33, 0, 0.7071068, 0, 0.7071068, QRInfoType.PosZ),
            new KiboQRField("2-1", 10.30, -7.5, 4.7, 0, 0, 1, 0, QRInfoType.QuaX),
            new KiboQRField("2-2", 11.5, -8, 5, 0, 0, 0, 1, QRInfoType.QuaY),
            new KiboQRField("2-3", 11, -7.7, 5.55, 0, -0.7071068, 0, 0.7071068, QRInfoType.QuaZ)
    ));

    private List<KeepZone> KZTable = new ArrayList<KeepZone>(Arrays.asList(
            new KeepZone(10.75, -4.9, 4.8, 10.95, -4.7, 5.0, false),
            new KeepZone(10.75, -6.5, 3.9, 11.95, -6.4, 5.9, false),
            new KeepZone(9.95, -7.2, 3.9, 10.85, -7.1, 5.9, false),
            new KeepZone(10.10, -8.6, 5.4, 11.1, -8.3, 5.9, false),
            new KeepZone(11.45, -9.0, 4.1, 11.95, -8.5, 5.1, false),
            new KeepZone(9.95, -9.1, 4.6, 10.45, -8.5, 5.1, false),
            new KeepZone(10.95, -8.4, 4.9, 11.15, -8.2, 5.1, false),
            new KeepZone(11.05, -8.9, 4.2, 11.25, -8.7, 4.4, false),
            new KeepZone(10.45, -9.1, 4.6, 10.65, -8.9, 4.8, false),
            new KeepZone(10.25, -9.75, 4.2, 11.65, -3, 5.6, true)
    ));
    public static final Boolean DOJAXA = false;
    public static final String LOGTAG = "TohokuKibo";
    public static final String EPOCK_UNIQUE_STR = UUID.randomUUID().toString().substring(4);

    private AstrobeeField AstrobeeNode = new AstrobeeField(10.95, -3.75, 4.85, 0, 0, 0.707, -0.707);

    @Override
    protected void runPlan1() {
        runPlan2();
//        runPlan3();

    }

    @Override
    protected void runPlan2() {
        Log.i(LOGTAG, "apijudgeSendStart");
        api.judgeSendStart();
        String p1_1_con[] = new String[2];
        String p1_2_con[] = new String[2];
        String p1_3_con[] = new String[2];
        String p2_1_con[] = new String[2];
        String p2_2_con[] = new String[2];
        String p2_3_con[] = new String[2];

        double px3 = 0,py3 = 0,pz3=0,qx3 = 0,qy3=0,qz3=0;
        int arv = 0;
        final int LOOPSIZE = 4;

        double distance = 0.00;
        double inc = 0.05;
        Vec3 road1_1_v = new Vec3(11.15, -4.8, 4.55);
        Vec3 target1_1_v = new Vec3(11.5 - distance, -5.7, 4.5);
        Vec3 target1_2_v = new Vec3(11, -6, 5.55 - distance - 0.1);
        Vec3 target1_3_v = new Vec3(11, -5.5, 4.33 + distance);


        WrapQuaternion road1_1_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion target1_3_q = new WrapQuaternion(0, 0.707f, 0, 0.707f);
        WrapQuaternion target1_1_q = new WrapQuaternion(0, 0, 0, -1);
        WrapQuaternion target1_2_q = new WrapQuaternion(0, -0.707f, 0, 0.707f);


        Vec3 road2_1_v = new Vec3(10.5, -6.45, 5.1);
        Vec3 road2_2_v = new Vec3(11.35, -7.3, 4.9);
        Vec3 target2_1_v = new Vec3(10.30 + distance, -7.5, 4.7);
        Vec3 target2_2_v = new Vec3(11.5 - distance, -8, 5);
        Vec3 target2_3_v = new Vec3(11, -7.7, 5.55 - distance);


        WrapQuaternion road2_1_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion road2_2_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion target2_1_q = new WrapQuaternion(0, 0, 1,0);
        WrapQuaternion target2_2_q = new WrapQuaternion(0, 0, 0,-1);
        WrapQuaternion target2_3_q = new WrapQuaternion(0, -0.707f, 0, 0.707f);

        moveTo(road1_1_v, road1_1_q);

        int loopCounter = 0;
        String p1_3 = "";
        p1_3 = scanBarcodeMoveTo(target1_3_v, target1_3_q, QRInfoType.PosZ);
        if (p1_3.equals("error")) {
            do {
                moveTo(target1_3_v.add(new Vec3(0, 0, -inc)), target1_3_q);
                p1_3 = scanBarcode(QRInfoType.PosZ);
                loopCounter++;
            } while (p1_3.equals("error") && loopCounter < LOOPSIZE);
            loopCounter = 0;
        }
        Log.d(LOGTAG, "p1_3 = " + p1_3);
        if (!p1_3.equals("error")) {
            p1_3_con = p1_3.split(", ");
            pz3 = Double.parseDouble(p1_3_con[1]);
        }

        String p1_1 = "";
        p1_1 = scanBarcodeMoveTo(target1_1_v, target1_1_q, QRInfoType.PosX);
        if (p1_1.equals("error")) {
            do {
                moveTo(target1_1_v.add(new Vec3(inc, 0, 0)), target1_1_q);
                p1_1 = scanBarcode(QRInfoType.PosX);
                loopCounter++;
            } while (p1_1.equals("error") && loopCounter < LOOPSIZE);
            loopCounter = 0;
        }
        Log.d(LOGTAG, "p1_1 = " + p1_1);
        if (!p1_1.equals("error")) {
            p1_1_con = p1_1.split(", ");
            px3 = Double.parseDouble(p1_1_con[1]);
        }

        String p1_2  = "";
        p1_2 = scanBarcodeMoveTo(target1_2_v, target1_2_q, QRInfoType.PosY);
        if (p1_2.equals("error")) {
            do {
                moveTo(target1_2_v.add(new Vec3(0, 0, inc)), target1_2_q);
                p1_2 = scanBarcode(QRInfoType.PosY);

                loopCounter++;
            } while (p1_2.equals("error") && loopCounter < LOOPSIZE);
            loopCounter = 0;
        }
        Log.d(LOGTAG, "p1_2 = " + p1_2);
        if (!p1_2.equals("error")) {
            p1_2_con = p1_2.split(", ");
            py3 = Double.parseDouble(p1_2_con[1]);
        }


        moveTo(road2_1_v, road2_1_q);
        moveTo(road2_2_v, road2_2_q);


        String p2_1  = "";
        p2_1 = scanBarcodeMoveTo(target2_1_v, target2_1_q, QRInfoType.QuaX);
        if (p2_1.equals("error")) {
            do {
                moveTo(target2_1_v.add(new Vec3(-inc, 0,0)), target2_1_q);
                p2_1 = scanBarcode(QRInfoType.QuaX);

                loopCounter++;
            } while (p2_1.equals("error") && loopCounter < LOOPSIZE);
            loopCounter = 0;
        }
        Log.d(LOGTAG, "p2_1 = " + p2_1);
        if (!p2_1.equals("error")) {
            p2_1_con = p2_1.split(", ");
            qx3 = Double.parseDouble(p2_1_con[1]);
        }

        String p2_2  = "";
        p2_2 = scanBarcodeMoveTo(target2_2_v, target2_2_q, QRInfoType.QuaY);
        if (p2_2.equals("error")) {
            do {
                moveTo(target2_2_v.add(new Vec3(inc, 0,0)), target2_2_q);
                p2_2 = scanBarcode(QRInfoType.QuaY);

                loopCounter++;
            } while (p2_2.equals("error") && loopCounter < LOOPSIZE);
            loopCounter = 0;
        }
        Log.d(LOGTAG, "p2_2 = " + p2_2);
        if (!p2_2.equals("error")) {
            p2_2_con = p2_2.split(", ");
            qy3 = Double.parseDouble(p2_2_con[1]);
        }

        String p2_3  = "";
        p2_3 = scanBarcodeMoveTo(target2_3_v, target2_3_q, QRInfoType.QuaZ);
        if (p2_2.equals("error")) {
            do {
                moveTo(target2_3_v.add(new Vec3(0, 0, inc)), target2_3_q);
                p2_3 = scanBarcode(QRInfoType.QuaZ);

                loopCounter++;
            } while (p2_3.equals("error") && loopCounter < LOOPSIZE);
            loopCounter = 0;
        }
        Log.d(LOGTAG, "p2_3 = " + p2_3);
        if (!p2_3.equals("error")) {
            p2_3_con = p2_3.split(", ");
            qz3 = Double.parseDouble(p2_3_con[1]);
        }

        moveTo(px3,py3,pz3,qx3,qy3,qz3,Math.sqrt(1 - (qx3 * qx3) - (qy3 * qy3) - (qz3 * qz3)));
        Mat ids = new Mat();
        while (arv == 0) {
            Mat source = api.getMatNavCam();
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            Aruco.detectMarkers(source, dictionary, corners, ids);
            arv = (int) ids.get(0, 0)[0];
        }

        api.judgeSendDiscoveredAR(Integer.toString(arv));
        api.laserControl(true);
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


    //-------------QR Decoding--------------------

    private String scanBarcode(QRInfoType type){
        Log.d(LOGTAG,"start scanBarcode");
        Mat snapshot = api.getMatNavCam();
        String value = detectQrcode(snapshot, type.name());
        if (value != "error") {
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
            if (value != "error") {
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
        while (value == "error" && loop < loopMax) {
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
        if (value != "error") {
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
        if (value != "error") {
            api.judgeSendDiscoveredQR(type.getInt() , value);
            Log.d(LOGTAG,"valuesQR" + value);
        }
        else{
            Log.d(LOGTAG,"valuesQR = error");
        }
        return value;
    }

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

    private String detectQrcode(Mat nmat, String key) {
        Log.d(LOGTAG,"start detectQrcode");
        if(nmat == null){
            Log.d(LOGTAG,"nmat == null");
            return "error";
        }

        ImageWrite(nmat, key);

        // 1:zxing decode
        try {
            Log.d(LOGTAG,"detectQrcode 1:zxing decode");
            String result = zxingDetectDecodeQrcode(nmat);
            if (!result.equals("error") || 0 < result.length()){
                return result;
            }
            Log.d(LOGTAG,"detectQrcode 2: qr rect base cutimage & decode");
        }catch (Exception e){
            Log.d(LOGTAG, "1:zxing decode exception detectQrcode: "+ e.getLocalizedMessage());
        }

        //2: qr rect base cutimage & decode
        List<Mat> points = rectTrimPoint(nmat);
        String result = "";
        if(points != null){
            Log.d(LOGTAG,"detectQrcode 2: points.size(): " + points.size());
            for(int i=0; i< points.size(); i++){
                Mat writemat = pointCuting(nmat, points.get(i));
                ImageWrite(writemat, key+"-pointCuting");
                writemat = sharpenFilter(writemat);
                ImageWrite(writemat, key+"-sharpenFilter");
                try {
                    result = zxingDetectDecodeQrcode(writemat);
                    if (!result.equals("error") || 0 < result.length()){
                        return result;
                    }
                }catch (Exception e){
                    Log.d(LOGTAG, "2: qr rect base cutimage & decode/exception detectQrcode: "+ e.getLocalizedMessage());
                }
            }
        }

        // 3:opencv try detect & decode
        try {
             Log.d(LOGTAG,"detectQrcode 3:opencv try detect & decode");
            Mat detectPoint = opencvDetectQrcode(nmat);
            result = "";
            if (detectPoint != null){
                Mat writemat  = pointCuting(nmat, detectPoint);
                ImageWrite(writemat, key+"-pointCuting");
                writemat = sharpenFilter(writemat);
                ImageWrite(writemat, key+"-sharpenFilter");

                if (!result.equals("error") || 0 < result.length()){
                    return result;
                }
                result = opencvDecodeDetectQrcode(writemat);
                if (!result.equals("error") || 0 < result.length()){
                    return result;
                }
            }
        }catch (Exception e){
            Log.d(LOGTAG, "3:opencv try detect & decode/exception detectQrcode: "+ e.getLocalizedMessage());
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
            // zxing で画像データを読み込み解析する
            Reader reader = new QRCodeReader();
            Map<DecodeHintType, Object> hintMap = new HashMap<DecodeHintType, Object>();
            hintMap.put(DecodeHintType.TRY_HARDER, Boolean.TRUE);

            com.google.zxing.Result decodeResult = reader.decode(binaryBitmap, hintMap);
//            ResultPoint[] resultPoints = decodeResult.getResultPoints();
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

    private Mat opencvDetectQrcode(Mat nmat) {
        if(nmat == null){
            Log.d(LOGTAG,"nmat == null");
            return null;
        }

        Log.d(LOGTAG, "opencvDetectQrcode try");

        QRCodeDetector detector = new QRCodeDetector();
        Mat point = new Mat();
        Boolean isDetect = detector.detect(nmat, point);
        Log.d(LOGTAG, "opencvDetectQrcode isDetect : " + isDetect);

        return point;
    }

    private String opencvDecodeDetectQrcode(Mat nmat) {
        if(nmat == null){
            Log.d(LOGTAG,"opencvDecodeDetectQrcode == null");
            return "error";
        }

        Log.d(LOGTAG, "opencvDecodeDetectQrcode try");

        try {
            QRCodeDetector detector = new QRCodeDetector();

            String result = detector.detectAndDecode(nmat);

            Log.d(LOGTAG,"opencvDecodeDetectQrcode"+result);
            return result;
        } catch (Exception e) {
            Log.d(LOGTAG, "exception opencvDecodeDetectQrcode"+ e.getLocalizedMessage());
            return "error";
        }
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

    private List<Mat> rectTrimPoint(Mat nmat){
        Log.d(LOGTAG,"rectTrimPoint try");

        Mat fmap = nmat.clone();

        Imgproc.threshold(fmap, fmap, 200, 255, Imgproc.THRESH_BINARY_INV);
        ImageWrite(fmap, "threshold1");

        Core.bitwise_not(fmap, fmap);
        ImageWrite(fmap, "bitwise_not");

        Mat hierarchy = Mat.zeros(new Size(5,5), CvType.CV_8UC1);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(fmap, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Log.d(LOGTAG,"rectTrimPoint contours.size: " + contours.size());

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
            Imgproc.approxPolyDP(contours2f, approx2f, ep * 0.04, true);

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


    private Mat tryMatNavCam() {
        final int LOOP_MAX = 3;

        Mat result = this.api.getMatNavCam();
        int loopCounter = 0;
        Log.i(MainService.LOGTAG, "tryMatNavCam Result: " + (result!=null));

        while (result == null && loopCounter < LOOP_MAX) {
            result = this.api.getMatNavCam();
            Log.i(MainService.LOGTAG, "tryMatNavCam Result: " + (result!=null));
            ++loopCounter;
        }
        return result;
    }

    //-----------------AR processing----------------------


    //-----------------basic functions----------------------

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

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion(qua_x, qua_y, qua_z, qua_w);

        String mes = "{" + "moveTo" + ","
                + BigDecimal.valueOf(pos_x).toPlainString() + ","
                + BigDecimal.valueOf(pos_y).toPlainString() + ","
                + BigDecimal.valueOf(pos_z).toPlainString() + ","
                + BigDecimal.valueOf(qua_x).toPlainString() + ","
                + BigDecimal.valueOf(qua_y).toPlainString() + ","
                + BigDecimal.valueOf(qua_z).toPlainString() + ","
                + BigDecimal.valueOf(qua_w).toPlainString() + ","
                + "}";

        Result result = this.api.moveTo(point, quaternion, true);
        int loopCounter = 0;
        Log.i(MainService.LOGTAG, "MoveTo Result: " + result.getStatus().toString());
        Log.i(MainService.LOGTAG, "MoveTo Do Params: " + mes);

        while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
            result = this.api.moveTo(point, quaternion, true);
            Log.i(MainService.LOGTAG, "MoveTo Result: " + result.getStatus().toString());
            Log.i(MainService.LOGTAG, "MoveTo Do Params: " + mes);
            ++loopCounter;
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
        String mes = "{" + "relativeMoveToRun" + ","
                + BigDecimal.valueOf(pos_x).toPlainString() + ","
                + BigDecimal.valueOf(pos_y).toPlainString() + ","
                + BigDecimal.valueOf(pos_z).toPlainString() + ","
                + BigDecimal.valueOf(qua_x).toPlainString() + ","
                + BigDecimal.valueOf(qua_y).toPlainString() + ","
                + BigDecimal.valueOf(qua_z).toPlainString() + ","
                + BigDecimal.valueOf(qua_w).toPlainString() + ","
                + "}";

        final int LOOP_MAX = 3;
        int loopCounter = 0;
        Result result = api.relativeMoveTo(point, quaternion, true);
        Log.i(MainService.LOGTAG, "relativeMoveTo Result: " + result.getStatus().toString());
        Log.i(MainService.LOGTAG, "relativeMoveTo Do Params: " + mes);

        while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
            result = this.api.relativeMoveTo(point, quaternion, true);
            Log.i(MainService.LOGTAG, "relativeMoveTo Result: " + result.getStatus().toString());
            Log.i(MainService.LOGTAG, "relativeMoveTo Do Params: " + mes);
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
}

