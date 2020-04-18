package jp.jaxa.iss.kibo.rpc.tohoku;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */


public class MainService extends KiboRpcService {

    private KiboQRField QRTable[] = {
            new KiboQRField("1-1", 11.5, -5.7, 4.5, 0, 0, 0, 1, QRInfoType.PosX),
            new KiboQRField("1-2", 11, -6, 5.55, 0, -0.7071068, 0, 0.7071068, QRInfoType.PosY),
            new KiboQRField("1-3", 11, -5.5, 4.33, 0, 0.7071068, 0, 0.7071068, QRInfoType.PosZ),
            new KiboQRField("2-1", 10.30, -7.5, 4.7, 0, 0, 1, 0, QRInfoType.QuaX),
            new KiboQRField("2-2", 11.5, -8, 5, 0, 0, 0, 1, QRInfoType.QuaY),
            new KiboQRField("2-3", 11, -7.7, 5.55, 0, -0.7071068, 0, 0.7071068, QRInfoType.QuaZ)
    };

    private Wrappers wraps = new Wrappers(api);

    @Override
    protected void runPlan1() {
        // start this run
        api.judgeSendStart();

        wraps.moveTo(10.6, -4.3, 5, 0, 0, -0.7071068, 0.7071068);
        wraps.moveTo(11, -4.3, 5, 0, 0, -0.7071068, 0.7071068);
        wraps.moveTo(11, -5.7, 5, 0, 0, -0.7071068, 0.7071068);
        wraps.moveTo(11.5, -5.7, 4.5, 0, 0, 0, 1);
        wraps.moveTo(11, -6, 5.55, 0, -0.7071068, 0, 0.7071068);

        wraps.moveTo(11.1, -6, 5.55, 0, -0.7071068, 0, 0.7071068);

//        Bitmap snapshot = api.getBitmapNavCam();
//        api.judgeSendDiscoveredAR(markerId);
        api.laserControl(true);
        api.judgeSendFinishSimulation();
//        sendData(MessageType.JSON, "data", "SUCCESS:defaultapk runPlan1");
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }



    private void runPlan1_init() {

    }
}

