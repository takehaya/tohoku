package jp.jaxa.iss.kibo.rpc.tohoku;

public class KeepZone{
    private double mXMin;
    private double mYMin;
    private double mZMin;
    private double mXMax;
    private double mYMax;
    private double mZMax;
    private Boolean isKiz;

    public KeepZone(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax, Boolean isKiz){
        this.mXMin = xmin;
        this.mYMin = ymin;
        this.mZMin = zmin;
        this.mXMax = xmax;
        this.mYMax = ymax;
        this.mZMax = zmax;
        this.isKiz = isKiz;
    }

    // TODO: Caculate strictly by taking into account the size of the HW
    // TODO: Judging by the direction of the object
    public Boolean isCheckScope(AstrobeeField astrobeeNode){
        if(this.isKiz){
            if( astrobeeNode.getPx() <= this.mXMin || astrobeeNode.getPx() >= this.mXMax ||
                astrobeeNode.getPy() <= this.mYMin || astrobeeNode.getPy() >= this.mYMax ||
                astrobeeNode.getPz() <= this.mZMin || astrobeeNode.getPz() >= this.mZMax ){
                return true;
            }
            return false;
        }
        // check koz
        if( astrobeeNode.getPx() >= this.mXMin && astrobeeNode.getPx() <= this.mXMax &&
            astrobeeNode.getPy() >= this.mYMin && astrobeeNode.getPy() <= this.mYMax &&
            astrobeeNode.getPz() >= this.mZMin && astrobeeNode.getPz() <= this.mZMax ){
            return true;
        }
        return false;
    }
}