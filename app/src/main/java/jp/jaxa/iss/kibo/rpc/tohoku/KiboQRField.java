package jp.jaxa.iss.kibo.rpc.tohoku;

enum QRInfoType {
    PosX(0,"pos_x"),
    PosY(1,"pos_y"),
    PosZ(2, "pos_z"),
    QuaX(3, "qua_x"),
    QuaY(4, "qua_y"),
    QuaZ(5, "qua_z"),
    P3(6, "p3"),
    ;

    private final int id;
    private final String key;
    QRInfoType(final int id, String key)
    {
        this.id = id;
        this.key = key;
    }

    public int getInt() {
        return this.id;
    }
    public String  getKey() {
        return this.key;
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