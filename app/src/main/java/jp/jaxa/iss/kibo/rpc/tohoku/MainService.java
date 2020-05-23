package jp.jaxa.iss.kibo.rpc.tohoku;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
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
        new KeepZone(10.10, -8.6, 5.4, 11.1, -83, 5.9, false),
        new KeepZone(11.45, -9.0, 4.1, 11.95, -8.5, 5.1, false),
        new KeepZone(9.95, -9.1, 4.6, 10.45, -8.5, 5.1, false),
        new KeepZone(10.95, -8.4, 4.9, 11.15, -8.2, 5.1, false),
        new KeepZone(11.05, -9.9, 4.2, 11.25, -8.7, 4.4, false),
        new KeepZone(10.45, -9.1, 4.6, 10.65, -8.9, 4.8, false),
        new KeepZone(10.25, -9.75, 4.2, 11.65, -3, 5.6, true)
    ));
    public static final String LOGTAG = "TohokuKibo";

    private AstrobeeField AstrobeeNode = new AstrobeeField(10.95, -3.75, 4.85, 0, 0, 0.707, -0.707);

    @Override
    protected void runPlan1() {
        runPlan2();
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
        final int LOOPSIZE = 3;


        Vec3 road1_1_v=new Vec3(11.15,-4.8,4.55);
        Vec3 target1_3_v=new Vec3(11,-5.5,4.55-0.2);
        Vec3 target1_1_v=new Vec3(11.3+0.2,-5.7,4.5);
        Vec3 target1_2_v=new Vec3(11,-6,5.35+0.3);

        WrapQuaternion road1_1_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion target1_3_q = new WrapQuaternion(0, 0.707f, 0, 0.707f);
        WrapQuaternion target1_1_q = new WrapQuaternion(0, 0, 0, -1);
        WrapQuaternion target1_2_q = new WrapQuaternion(0, -0.707f, 0, 0.707f);

        Vec3 road2_1_v = new Vec3(10.5,-6.45,5.1);
        Vec3 road2_2_v = new Vec3(11.35,-7.2,4.9);
        Vec3 target2_1_v = new Vec3(10.48,-7.5,4.7);
        Vec3 target2_3_v = new Vec3(11,-7.7,5.37);
        Vec3 target2_2_v = new Vec3(11.32,-8,5);

        WrapQuaternion road2_1_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion road2_2_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion target2_1_q = new WrapQuaternion(0, 0, 0,1);
        WrapQuaternion target2_2_q = new WrapQuaternion(0, 0, 0,-1);
        WrapQuaternion target2_3_q = new WrapQuaternion(0, -0.707f, 0, 0.707f);

        moveTo(road1_1_v, road1_1_q);

        int loopCounter = 0;
        String p1_3 = "";
        do{
            moveTo(target1_3_v, target1_3_q);
            p1_3 = scanBarcode(QRInfoType.PosZ);
            Log.d(LOGTAG,"p1_3 = " + p1_3);
            if (p1_3 != "error"){
                p1_3_con = p1_3.split(", ");
                pz3 = Double.parseDouble(p1_3_con[1]);
            }
            loopCounter++;
        }while (p1_3.equals("error") &&loopCounter<LOOPSIZE);

        loopCounter = 0;

        String p1_1 = "";
        do{
            moveTo(target1_1_v, target1_1_q);
            p1_1 = scanBarcode(QRInfoType.PosX);

            Log.d(LOGTAG,"p1_1 = " + p1_1);
            if (p1_1 != "error"){
                p1_1_con = p1_1.split(", ");
                px3 = Double.parseDouble(p1_1_con[1]);
            }
            loopCounter++;
        }while (p1_1.equals("error") &&loopCounter<LOOPSIZE);
        loopCounter = 0;


        String p1_2  = "";
        do{
            moveTo(target1_2_v.add(new Vec3(0,0,0.05)), target1_2_q);
            p1_2 =  scanBarcode(QRInfoType.PosY);

            Log.d(LOGTAG,"p1_2 = " + p1_2);
            if (p1_2 != "error"){
                p1_2_con = p1_2.split(", ");
                py3 = Double.parseDouble(p1_2_con[1]);
            }
            loopCounter++;
        }while (p1_2.equals("error") &&loopCounter<LOOPSIZE);
        loopCounter = 0;


        moveTo(road2_1_v, road2_1_q);
        moveTo(road2_2_v, road2_2_q);


        String p2_1  = "";
        do{
            moveTo(target2_1_v, target2_1_q);
            p2_1 = scanBarcode(QRInfoType.QuaX);
            Log.d(LOGTAG,"p2_1 = " + p2_1);
            if (p2_1 != "error"){
                p2_1_con = p2_1.split(", ");
                qx3 = Double.parseDouble(p2_1_con[1]);
            }
            loopCounter++;
        }while (p2_1.equals("error") &&loopCounter<LOOPSIZE);
        loopCounter = 0;


        String p2_2  = "";
        do{
            moveTo(target2_2_v, target2_2_q);
            p2_2 = scanBarcode(QRInfoType.QuaY);
            Log.d(LOGTAG,"p2_2 = " + p2_2);
            if (p2_2 != "error"){
                p2_2_con = p2_2.split(", ");
                qy3 = Double.parseDouble(p2_2_con[1]);
            }
            loopCounter++;
        }while (p2_2.equals("error") &&loopCounter<LOOPSIZE);
        loopCounter = 0;


        String p2_3  = "";
        do{
            moveTo(target2_3_v, target2_3_q);
            p2_3 = scanBarcode(QRInfoType.QuaZ);
            Log.d(LOGTAG,"p2_3 = " + p2_3);
            if (p2_3 != "error"){
                p2_3_con = p2_3.split(", ");
                qz3 = Double.parseDouble(p2_3_con[1]);
            }

            loopCounter++;
        }while (p2_3.equals("error") &&loopCounter<LOOPSIZE);
        loopCounter = 0;


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
        if (mat == null){
            return false;
        }
        Bitmap bitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, bitmap);
        String uniqueString = UUID.randomUUID().toString();
        try {
            File path = getExternalFilesDir(Environment.DIRECTORY_PICTURES);
            File file = new File(path, uniqueString);
            FileOutputStream out = new FileOutputStream(file  + "-" + key + ".png");
            Boolean ok = bitmap.compress(Bitmap.CompressFormat.PNG, 90, out);
            Log.d(LOGTAG, "ImageWrite: "+ file + "-" + key + ".png");
            Log.d(LOGTAG, "ImageWrite status: "+ ok);
            out.close();

            return ok;
        } catch (Exception e){
            Log.d(LOGTAG, "ImageWrite fail: "+ e.getLocalizedMessage());
        }
        return false;
    }

    private String detectQrcode(Mat nmat, String key) {
        Log.d(LOGTAG,"start detectQrcode");
        if(nmat == null){
            Log.d(LOGTAG,"nmat == null");
            return "error";
        }
        final int loopsize = 2;
        for(int i = 0; i< loopsize; i++){
            ImageWrite(nmat, key);
            try {
                String result = zxingDetectDecodeQrcode(nmat);
                if (result.equals("error")){
                    Mat detectPoint = opencvDetectQrcode(nmat);
                    if (detectPoint != null ){
                        nmat = pointCuting(nmat, detectPoint);
                        result = opencvDecodeQrcode(nmat, detectPoint);
                        if (result.equals("error")){
                            continue;
                        }
                    }else{
                        Mat point = rectTrimPoint(nmat);
                        nmat = pointCuting(nmat, point);
                        nmat = sharpenFilter(nmat);
                        continue;
                    }
                }
                return result;
            }catch (Exception e){
                Log.d(LOGTAG, "exception detectQrcode: "+ e.getLocalizedMessage());
            }
        }

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
        if (!isDetect){
            return null;
        }
        return point;
    }

    private String opencvDecodeQrcode(Mat nmat, Mat point) {
        if(nmat == null || point == null){
            Log.d(LOGTAG,"nmat or point == null");
            return "error";
        }

        Log.d(LOGTAG, "opencvDetectQrcode try");

        try {
            QRCodeDetector detector = new QRCodeDetector();

            String result = detector.decode(nmat, point);
            if(!(0 < result.length()) || result == null) {
                Mat optmat = preQRProcessing(nmat);

                if(optmat == null){
                    Log.d(LOGTAG,"optmat == null");
                    return "error";
                }
                result = detector.detectAndDecode(optmat);
                if(!(0 < result.length()) || result == null) {
                    return "error";
                }
            }
            Log.d(LOGTAG,"opencvDecodeQrcode"+result);
            return result;
        } catch (Exception e) {
            Log.d(LOGTAG, "exception opencvDecodeQrcode"+ e.getLocalizedMessage());
            return "error";
        }
    }

    private Mat pointCuting(Mat nmat, Mat point){
        Log.d(LOGTAG,"pointCuting try");

        Mat mat = new Mat(400,400,CvType.CV_32F);
        float distPoint[] = new float[]{0,400, 0,0 ,400,0 ,400,400};//Lower left, upper left, upper right, lower left.
        Mat dstmat = new Mat(4,2, CvType.CV_32F);
        dstmat.put(0,0, distPoint);
        Log.d(LOGTAG,"dstmat.put done");
        Log.d(LOGTAG,"point dump: " + point.dump());
        Log.d(LOGTAG,"point depth: " + point.depth());
        Log.d(LOGTAG,"dstmat dump: " + dstmat.dump());
        Log.d(LOGTAG,"dstmat depth: " + dstmat.depth());


        Mat rmat = Imgproc.getPerspectiveTransform(point, dstmat);
        Log.d(LOGTAG,"getPerspectiveTransform done");

        Imgproc.warpPerspective(nmat, mat, rmat, mat.size());
        Log.d(LOGTAG,"warpPerspective done");

        return mat;
    }

    private Mat rectTrimPoint(Mat nmat){
        Log.d(LOGTAG,"rectTrimPoint try");

        Mat fmap = new Mat();

        Imgproc.cvtColor(nmat, fmap, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(fmap, fmap, 200, 266, Imgproc.THRESH_TOZERO_INV);
        Core.bitwise_not(fmap, fmap);
        Imgproc.threshold(fmap, fmap, 0, 255, Imgproc.THRESH_BINARY_INV|Imgproc.THRESH_OTSU);

        Mat hierarchy = Mat.zeros(new Size(5,5), CvType.CV_8UC1);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(fmap, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_TC89_L1);

        for(int i = 0; i<contours.size(); i++){
            MatOfPoint ptmat = contours.get(i);
            double size = Imgproc.contourArea(ptmat);
            if (size < nmat.size().area() / (100 * 1)) {
                /* サイズが小さいエリアは無視 */
                continue;
            }
            MatOfPoint2f ptmat2f = new MatOfPoint2f(ptmat.toArray());
            MatOfPoint2f approx = new MatOfPoint2f();
            MatOfPoint approxf1 = new MatOfPoint();
            double ep = Imgproc.arcLength(ptmat2f, true);
            Imgproc.approxPolyDP(ptmat2f, approx, ep * 0.03, true);

            approx.convertTo(approxf1, CvType.CV_32S);
            if(approx.size().area() == 4){
                return approxf1;
            }
        }
        return null;
    }

    private Mat sharpenFilter(Mat nmat){
        final int k = -1;
        Mat kernel = new Mat(3, 3, -1);
        kernel.put(0, 0, new float[]{k,k,k});
        kernel.put(1, 0, new float[]{k,9,k});
        kernel.put(2, 0, new float[]{k,k,k});
        Mat destination=new Mat(nmat.rows(),nmat.cols(),nmat.type());
        Imgproc.filter2D(nmat, destination, -1, kernel);

        return destination;
    }
//    private String detectQrcode(Mat nmat, String key) {
//        Log.d(LOGTAG,"start detectQrcode");
//
//        if(nmat == null){
//            Log.d(LOGTAG,"nmat == null");
//            return "error";
//        }
//
//        ImageWrite(nmat, key);
//        Mat optmat = preQRProcessing(nmat);
//        ImageWrite(optmat, key);
//
//        if(optmat == null){
//            Log.d(LOGTAG,"optmat == null");
//            optmat = nmat;
//        }else{
//            Log.d(LOGTAG, "optmap:" + optmat.dump());
//        }
//
////        Imgproc.medianBlur(mat, mat, 5);
////        Imgproc.adaptiveThreshold(mat, mat, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY, 11, 2);
//        Bitmap bitmap = Bitmap.createBitmap(optmat.width(), optmat.height(), Bitmap.Config.ARGB_8888);
//        Utils.matToBitmap(optmat, bitmap);
//        int width = bitmap.getWidth();
//        int height = bitmap.getHeight();
//
//        int[] pixels = new int[width * height];
//        bitmap.getPixels(pixels, 0, width, 0, 0, width, height);
//        Log.d(LOGTAG, "detectQrcode try");
//
//        try {
//            // zxing で扱える BinaryBitmap形式に変換する
//            LuminanceSource source = new RGBLuminanceSource(width, height, pixels);
//            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
//            // zxing で画像データを読み込み解析する
//            Reader reader = new QRCodeReader();
//            Map<DecodeHintType, Object> hintMap = new HashMap<DecodeHintType, Object>();
//            hintMap.put(DecodeHintType.TRY_HARDER, Boolean.TRUE);
//
//            com.google.zxing.Result decodeResult = reader.decode(binaryBitmap, hintMap);
//            // 解析結果を取得する
//            String result = decodeResult.getText();
//            if(!(0 < result.length()) || result == null) {
//                return "error";
//            }
//            Log.d(LOGTAG,"readQR"+result);
//            return result;
//        } catch (Exception e) {
//            Log.d(LOGTAG, "exception readQR"+ e.getLocalizedMessage());
//            return "error";
//        }
//    }
    private static Mat preQRProcessing(Mat mat){
        QRCodeDetector detector = new QRCodeDetector();

        Mat point = new Mat();
        Boolean isDetect = detector.detect(mat, point);
        Log.d(LOGTAG,"preQRProcessing isDetect point: "+point.dump());

        Log.i(LOGTAG, "preQRProcessing isDetect: " + isDetect);
        if (isDetect){
            Mat nmat = new Mat(400,400,CvType.CV_8UC3);
            float distPoint[] = new float[]{0,400, 0,0 ,400,0 ,400,400};//Lower left, upper left, upper right, lower left.
            Mat dstmat = new Mat(4,2, CvType.CV_32F);
            dstmat.put(0,0, distPoint);

            Mat rmat = Imgproc.getPerspectiveTransform(point, dstmat);
            Imgproc.warpPerspective(mat, nmat, rmat, nmat.size());

            return nmat;
        }
        return null;
    }


    private Mat tryMatNavCam() {
        final int LOOP_MAX = 3;

        Mat result = this.api.getMatNavCam();
        int loopCounter = 0;
        Log.i(MainService.LOGTAG, "tryMatNavCam Result: " + !(result!=null));

        while (result == null && loopCounter < LOOP_MAX) {
            result = this.api.getMatNavCam();
            Log.i(MainService.LOGTAG, "tryMatNavCam Result: " + !(result!=null));
            ++loopCounter;
        }
        return result;
    }

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
}

