package jp.jaxa.iss.kibo.rpc.tohoku;

enum QRInfoType {
    PosX(0),
    PosY(1),
    PosZ(2),
    QuaX(3),
    QuaY(4),
    QuaZ(5),
    P3(6),
    ;

    private final int id;

    private QRInfoType(final int id) {
        this.id = id;
    }

    public int getInt() {
        return this.id;
    }
    public static QRInfoType getType(final int id) {
        QRInfoType[] types = QRInfoType.values();
        for (QRInfoType type : types) {
            if (type.getInt() == id) {
                return type;
            }
        }
        return null;
    }
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