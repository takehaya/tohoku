package jp.jaxa.iss.kibo.rpc.tohoku;

import org.opencv.core.Size;
import org.opencv.objdetect.QRCodeDetector;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import java.util.HashMap;
import java.util.Map;

public class QRManager {
    private QRCodeDetector QRdetector;
    private Map<String, Double> QRState;

    public QRManager(){
        this.QRdetector = new QRCodeDetector();
        this.QRState = new HashMap<>();
    }

    public Map<String, Double> DecodeinTable(Mat input){
//        Imgproc.GaussianBlur(input, input, new Size(5, 5), 0, 0);
//        Imgproc.threshold(input, input, 0.0, 255.0, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);
        String data = this.Decoder(input);
        if (0 < data.length()){

            String[] temp = data.split(" ");
            QRState.put(temp[0], Double.parseDouble(temp[1]));

            return this.QRState;
        }
        return null;
    }

    public String Decoder(Mat input){
        return this.QRdetector.detectAndDecode(input);
    }

    public KiboQRField MakeP3(){
        double px, py, pz, ox, oy, oz;
        try {
            px = this.QRState.get("pos_x");
            py = this.QRState.get("pos_y");
            pz = this.QRState.get("pos_z");
            ox = this.QRState.get("qua_x");
            oy = this.QRState.get("qua_y");
            oz = this.QRState.get("qua_z");
            return new KiboQRField("3", px, py, pz, ox, oy, oz, 0, QRInfoType.P3);
        }catch (Exception e){
            return null;
        }
    }
}
