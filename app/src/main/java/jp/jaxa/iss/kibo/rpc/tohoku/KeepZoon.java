package jp.jaxa.iss.kibo.rpc.tohoku;

public class KeepZoon{
    private double mXMin;
    private double mYMin;
    private double mZMin;
    private double mXMax;
    private double mYMax;
    private double mZMax;
    private Boolean isKiz;

    public KeepZoon(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax, Boolean isKiz){
        this.mXMin = xmin;
        this.mYMin = ymin;
        this.mZMin = zmin;
        this.mXMax = xmax;
        this.mYMax = ymax;
        this.mZMax = zmax;
        this.isKiz = isKiz;
    }

    private double getXSize(){
        return this.mXMax - this.mXMin;
    }

    private double getYSize(){
        return this.mYMax - this.mYMin;
    }

    private double getZSize(){
        return this.mZMax - this.mZMin;
    }

    public Boolean CheckScope(KiboObject astrobeeNode ){
        if(this.isKiz){
            // check iskiz. inner range true
            return false;
        }

        return false;
    }
}