package jp.jaxa.iss.kibo.rpc.tohoku;

public class AstrobeeField extends KiboObject {
    final private double HWSize = 0.3;
    final private double HWSizeOffset = (HWSize*Math.sqrt(3))/2;

    public AstrobeeField(double px, double py, double pz, double ox, double oy, double oz, double ow) {
        super("astrobee", px, py, pz, ox, oy, oz, ow);

    }

    public double getHWSizeOffset(){return this.HWSizeOffset;}
}
