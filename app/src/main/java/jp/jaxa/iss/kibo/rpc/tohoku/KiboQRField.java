package jp.jaxa.iss.kibo.rpc.tohoku;

enum QRInfoType {
    PosX,
    PosY,
    PosZ,
    QuaX,
    QuaY,
    QuaZ,
}


public class KiboQRField extends KiboObject{
    private QRInfoType info;
    private double p3pram;

    public KiboQRField(String name, double px, double py, double pz, double ox, double oy, double oz, double ow, QRInfoType info) {
        super(name, px, py, pz, ox, oy, oz, ow);
        this.info = info;
        this.p3pram = 0;
    }

    public QRInfoType getInfo() {
        return info;
    }

    public void setInfo(QRInfoType mInfo) {
        info = mInfo;
    }

    public double getP3pram() {
        return p3pram;
    }

    public void setP3pram(double mP3pram) {
        p3pram = mP3pram;
    }
}