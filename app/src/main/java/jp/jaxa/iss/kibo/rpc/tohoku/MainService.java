package jp.jaxa.iss.kibo.rpc.tohoku;

import android.graphics.Bitmap;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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
//        // start this run
//        api.judgeSendStart();
//        Log.i(LOGTAG, "apijudgeSendStart");
//        //qr1 move
//        wraps.moveTo(10.6, -4.3, 5, 0, 0, -0.7071068, 0.7071068);
//        wraps.moveTo(10.6, -4.3, 5, 0, 0, -0.7071068, 0.7071068f);
//        wraps.moveTo(11, -4.3, 5, 0, 0, -0.7071068, 0.7071068);
//        wraps.moveTo(11, -5.7, 5, 0, 0,  -0.7071068, 0.7071068f);
//        wraps.moveTo(11.5, -5.7, 4.5, 0, 0, 0, 1);
//        Bitmap snapshot = api.getBitmapNavCam();
//        String QRcodeString = readQrcode(snapshot);
//        api.judgeSendDiscoveredQR(0,QRcodeString);//送信
//
////        api.judgeSendDiscoveredAR(markerId);
////        api.laserControl(true);
//        api.judgeSendFinishSimulation();
////        sendData(MessageType.JSON, "data", "SUCCESS:defaultapk runPlan1");
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


        Vec3 road1_1_v=new Vec3(11.15,-4.8,4.55);
        Vec3 target1_3_v=new Vec3(11,-5.5,4.55-0.2);
        Vec3 target1_1_v=new Vec3(11.3+0.1,-5.7,4.5);
        Vec3 target1_2_v=new Vec3(11,-6,5.35+0.1);

        WrapQuaternion road1_1_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion target1_3_q = new WrapQuaternion(0, 0.707f, 0, 0.707f);
        WrapQuaternion target1_1_q = new WrapQuaternion(0, 0, 0, -1);
        WrapQuaternion target1_2_q = new WrapQuaternion(0, -0.707f, 0, 0.707f);

        moveTo(road1_1_v, road1_1_q);
        moveTo(target1_3_v, target1_3_q);
        String p1_3 = scanBarcodeRelation(target1_3_q, QRInfoType.PosZ);
        Log.d(LOGTAG,"p1_3 = " + p1_3);
        if (p1_3 != "error"){
            p1_3_con = p1_3.split(", ");
            pz3 = Double.parseDouble(p1_3_con[1]);
        }

        moveTo(target1_1_v, target1_1_q);
        String p1_1 = scanBarcodeRelation(target1_1_q, QRInfoType.PosX);
        Log.d(LOGTAG,"p1_1 = " + p1_1);
        if (p1_1 != "error"){
            p1_1_con = p1_1.split(", ");
            px3 = Double.parseDouble(p1_1_con[1]);
        }

        moveTo(target1_2_v, target1_2_q);
        String p1_2 =  scanBarcodeRelation(target1_2_q, QRInfoType.PosY);
        Log.d(LOGTAG,"p1_2 = " + p1_2);
        if (p1_2 != "error"){
            p1_2_con = p1_2.split(", ");
            py3 = Double.parseDouble(p1_2_con[1]);
        }

        Vec3 road2_1_v=new Vec3(10.5,-6.45,5.1);
        Vec3 road2_2_v=new Vec3(11.35,-7.2,4.9);
        Vec3 target2_1_v=new Vec3(10.48,-7.5,4.7);
        Vec3 target2_3_v=new Vec3(11,-7.7,5.37);
        Vec3 target2_2_v=new Vec3(11.32,-8,5);

        WrapQuaternion road2_1_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion road2_2_q = new WrapQuaternion(0, 0, 0.707f, -0.707f);
        WrapQuaternion target2_1_q = new WrapQuaternion(0, 0, 0,1);
        WrapQuaternion target2_2_q = new WrapQuaternion(0, 0, 0,-1);
        WrapQuaternion target2_3_q = new WrapQuaternion(0, -0.707f, 0, 0.707f);

        moveTo(road2_1_v, road2_1_q);
        moveTo(road2_2_v, road2_2_q);
        moveTo(target2_1_v, target2_1_q);
        String p2_1 = scanBarcodeRelation(target2_1_q, QRInfoType.QuaX);
        Log.d(LOGTAG,"p2_1 = " + p2_1);
        if (p2_1 != "error"){
            p2_1_con = p2_1.split(", ");
            qx3 = Double.parseDouble(p2_1_con[1]);
        }

        moveTo(target2_3_v, target2_2_q);
        String p2_2 = scanBarcodeRelation(target2_2_q, QRInfoType.QuaY);
        Log.d(LOGTAG,"p2_2 = " + p2_2);
        if (p2_2 != "error"){
            p2_2_con = p2_2.split(", ");
            qy3 = Double.parseDouble(p2_2_con[1]);
        }

        moveTo(target2_2_v, target2_3_q);
        String p2_3 = scanBarcodeRelation(target2_3_q, QRInfoType.QuaZ);
        Log.d(LOGTAG,"p2_3 = " + p2_3);
        if (p2_3 != "error"){
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
    }


    //-------------QR Decoding--------------------

    private String scanBarcode(QRInfoType type){
        Log.d(LOGTAG,"start scanBarcode");
        Mat snapshot = api.getMatNavCam();
        String value = detectQrcode(snapshot);
        if (value != "error") {
            api.judgeSendDiscoveredQR(type.getInt() , value);
            Log.d(LOGTAG,"valuesQR" + value);
        }
        else{
            Log.d(LOGTAG,"valuesQR = error");
        }
        return value;
    }

    private String scanBarcodeMoveTo(Vec3 vec, WrapQuaternion qua,  QRInfoType type){
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
        String value = detectQrcode(snapshot);
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
            value = detectQrcode(snapshot);
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
        String value = detectQrcode(snapshot);
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
            value = detectQrcode(snapshot);
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


    private String detectQrcode(Mat mat) {
        if(mat == null){
            return "error";
        }

        Imgproc.GaussianBlur(mat, mat, new Size(mat.width(), mat.height()), 0);
        Imgproc.threshold(mat, mat, 0.0, 255.0,
                Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);
        Bitmap bitmap = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, bitmap);
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();

        int[] pixels = new int[width * height];
        bitmap.getPixels(pixels, 0, width, 0, 0, width, height);
        Log.d(LOGTAG, "detectQrcode try");

        try {
            // zxing で扱える BinaryBitmap形式に変換する
            LuminanceSource source = new RGBLuminanceSource(width, height, pixels);
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
            // zxing で画像データを読み込み解析する
            Reader reader = new QRCodeReader();
            com.google.zxing.Result decodeResult = reader.decode(binaryBitmap);
            // 解析結果を取得する
            String result = decodeResult.getText();
            if(!(0 < result.length())) {
                return "error";
            }
            Log.d(LOGTAG,"readQR"+result);
            return result;
        } catch (Exception e) {
            Log.d(LOGTAG, "readQR"+ e.getLocalizedMessage());
            return "error";
        }
    }

    private Mat tryMatNavCam() {
        final int LOOP_MAX = 3;

        Mat result = this.api.getMatNavCam();;
        int loopCounter = 0;
        while (result == null || loopCounter < LOOP_MAX) {
            result = this.api.getMatNavCam();
            ++loopCounter;
        }
        return result;
    }

    //-----------------basic functions----------------------

    public Result moveTo(Vec3 vec, WrapQuaternion quat) {
        return moveToRun(
                vec.getX(), vec.getY(), vec.getZ(),
                quat.getX(),quat.getY(), quat.getZ(), quat.getW()
        );
    }
    public Result moveTo(double pos_x, double pos_y, double pos_z,
                       double qua_x, double qua_y, double qua_z, double qua_w) {
        return moveToRun(
                pos_x, pos_y, pos_z,
                (float) qua_x, (float)qua_y, (float)qua_z, (float)qua_w
        );
    }
    private Result moveToRun(double pos_x, double pos_y, double pos_z,
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

        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            result = this.api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
        return result;
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

        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            result = this.api.relativeMoveTo(point, quaternion, true);
            ++loopCounter;
        }
        return result.getStatus();
    }
}

