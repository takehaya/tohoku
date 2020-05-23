package jp.jaxa.iss.kibo.rpc.tohoku;

import android.util.Log;

import java.math.BigDecimal;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

public class Wrappers {
    private KiboRpcApi api;
    public Wrappers(KiboRpcApi api) {
        this.api = api;
    }

    public void moveTo(Vec3 vec, WrapQuaternion quat) {
        moveToRun(
                vec.getX(), vec.getY(), vec.getZ(),
                quat.getX(),quat.getY(), quat.getZ(), quat.getW()
        );
    }
    public void moveTo(double pos_x, double pos_y, double pos_z,
                       double qua_x, double qua_y, double qua_z, double qua_w) {
        moveToRun(
                pos_x, pos_y, pos_z,
                (float) qua_x, (float)qua_y, (float)qua_z, (float)qua_w
        );
    }
    // You can add your method
    private void moveToRun(double pos_x, double pos_y, double pos_z,
                               float qua_x, float qua_y, float qua_z, float qua_w) {

        final int LOOP_MAX = 3;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        String mes = "{" + "MoveTo" + ","
                + BigDecimal.valueOf(pos_x).toPlainString() + ","
                + BigDecimal.valueOf(pos_y).toPlainString() + ","
                + BigDecimal.valueOf(pos_z).toPlainString() + ","
                + BigDecimal.valueOf(qua_x).toPlainString() + ","
                + BigDecimal.valueOf(qua_y).toPlainString() + ","
                + BigDecimal.valueOf(qua_z).toPlainString() + ","
                + BigDecimal.valueOf(qua_w).toPlainString() + ","
                + "}";
        Log.i(MainService.LOGTAG, mes);

        Result result = this.api.moveTo(point, quaternion, true);
        int loopCounter = 0;
        while (!result.hasSucceeded() || loopCounter < LOOP_MAX) {
            result = this.api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    public void moveToRelative(Vec3 vec, WrapQuaternion quat) {
        moveToRelativeRun(
                vec.getX(), vec.getY(), vec.getZ(),
                quat.getX(),quat.getY(), quat.getZ(), quat.getW()
        );
    }
    public void moveToRelative(double pos_x, double pos_y, double pos_z,
                       float qua_x, float qua_y, float qua_z, float qua_w) {
        moveToRelativeRun(
                pos_x, pos_y, pos_z,
                qua_x, qua_y, qua_z, qua_w
        );
    }

    private void moveToRelativeRun(double pos_x, double pos_y, double pos_z,
                                float qua_x, float qua_y, float qua_z, float qua_w){
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        api.relativeMoveTo(point, quaternion, true);
    }
    public void moveTo(Point pos,Quaternion qua){
        this.moveTo(pos.getX(),pos.getY(),pos.getZ(),qua.getX(),qua.getY(),qua.getZ(),qua.getW());
    }
}
