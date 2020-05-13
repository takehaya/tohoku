package jp.jaxa.iss.kibo.rpc.tohoku;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

public class Wrappers {
    private KiboRpcApi api;
    public Wrappers(KiboRpcApi api) {
        this.api = api;
    }

    // You can add your method
    public void moveTo(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w) {

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = this.api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            result = this.api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }
}
