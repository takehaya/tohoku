package jp.jaxa.iss.kibo.rpc.tohoku;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.fail;
import static org.junit.Assert.assertEquals;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.util.Map;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.ColorMatrix;
import android.graphics.ColorMatrixColorFilter;
import android.graphics.Paint;
import android.os.Build;
import android.util.Log;

import org.junit.runner.RunWith;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Core;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

@RunWith(RobolectricTestRunner.class)
@Config(constants=BuildConfig.class, sdk=25)
public class QRManagerTest {
    QRManager manager;
    Mat QR11;
    Mat QR12;
    Mat QR13;
    Mat QR21;
    Mat QR22;
    Mat QR23;

    static{
        System.load("/usr/local/share/OpenCV/java/libopencv_java344.so");
    }

    @Before
    public void setUp() throws Exception {
        this.manager = new QRManager();
        String basepath = System.getProperty("user.dir") + "/src/test";
        this.QR11 = getMatImage(basepath + "/res/textures/qr1_1.png");
        this.QR12 = getMatImage(basepath + "/res/textures/qr1_2.png");
        this.QR13 = getMatImage(basepath + "/res/textures/qr1_3.png");
        this.QR21 = getMatImage(basepath + "/res/textures/qr2_1.png");
        this.QR22 = getMatImage(basepath + "/res/textures/qr2_2.png");
        this.QR23 = getMatImage(basepath + "/res/textures/qr2_3.png");
    }

    private Mat getMatImage(String path) throws IOException {
        Mat image = Imgcodecs.imread(path);
        Imgproc.cvtColor(image, image, Imgproc.COLOR_RGB2GRAY);
        return image;
    }
    private Bitmap toBitmapGrayscale(Bitmap bmpOriginal) {
        int width, height;
        height = bmpOriginal.getHeight();
        width = bmpOriginal.getWidth();

        Bitmap bmpGrayscale = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        Canvas c = new Canvas(bmpGrayscale);
        Paint paint = new Paint();
        ColorMatrix cm = new ColorMatrix();
        cm.setSaturation(0);
        ColorMatrixColorFilter f = new ColorMatrixColorFilter(cm);
        paint.setColorFilter(f);
        c.drawBitmap(bmpOriginal, 0, 0, paint);
        return bmpGrayscale;
    }

    // TODO: It's the mock that bugs me.
    private Bitmap getBitmapImage(String path) throws IOException {
        InputStream inputStream = new FileInputStream(path);
        Bitmap image = BitmapFactory.decodeStream(inputStream);
        return toBitmapGrayscale(image);
    }
    public static int[] toIntArray(byte buf[])
    {
        final ByteBuffer buffer = ByteBuffer.wrap(buf)
                .order(ByteOrder.LITTLE_ENDIAN);
        final int[] ret = new int[buf.length / 4];
        buffer.asIntBuffer().put(ret);
        return ret;
    }

    @Test
    public void TestCase1() {
        Map<String, Double> state;
        state = this.manager.DecodeinTable(this.QR11);
        assertEquals("manger.decoder testing: Normal scenario", null,state.get("pos_x"));
        state = this.manager.DecodeinTable(this.QR12);
        state = this.manager.DecodeinTable(this.QR13);
//        assertEquals("manger.decoder length testing: Normal scenario", 3,state.size());

        assertEquals("manger.decoder return val testing: Normal scenario", null,state);
        state = this.manager.DecodeinTable(this.QR21);
        state = this.manager.DecodeinTable(this.QR22);
        state = this.manager.DecodeinTable(this.QR23);
        assertEquals("manger.decoder length testing: Normal scenario", 6,state.size());
        assertEquals("manger.decoder return val testing: Normal scenario", null,state);

        KiboQRField p3field = this.manager.MakeP3();
        assertEquals("makeP3 testing: Normal scenario", null,this.manager.MakeP3());

    }
}
