package jp.jaxa.iss.kibo.rpc.tohoku;

public class KiboObject{
    String name;
    private double mPx;
    private double mPy;
    private double mPz;
    private double mOx;
    private double mOy;
    private double mOz;
    private double mOw;

    public KiboObject(String name, double px, double py, double pz, double ox, double oy, double oz, double ow) {
        this.name = name;

        this.mPx = px;
        this.mPy = py;
        this.mPz = pz;
        this.mOx = ox;
        this.mOy = oy;
        this.mOz = oz;
        this.mOw = ow;
    }

    public String getName() {
        return name;
    }

    public void setName(String mName) {
        name = mName;
    }

    public double getPx() {
        return mPx;
    }

    public void setPx(double mPx) {
        this.mPx = mPx;
    }

    public double getPy() {
        return mPy;
    }

    public void setPy(double mPy) {
        this.mPy = mPy;
    }

    public double getPz() {
        return mPz;
    }

    public void setPz(double mPz) {
        this.mPz = mPz;
    }

    public double getOx() {
        return mOx;
    }

    public void setOx(double mOx) {
        this.mOx = mOx;
    }

    public double getOy() {
        return mOy;
    }

    public void setOy(double mOy) {
        this.mOy = mOy;
    }

    public double getOz() {
        return mOz;
    }

    public void setOz(double mOz) {
        this.mOz = mOz;
    }

    public double getOw() {
        return mOw;
    }

    public void setOw(double mOw) {
        this.mOw = mOw;
    }
}
