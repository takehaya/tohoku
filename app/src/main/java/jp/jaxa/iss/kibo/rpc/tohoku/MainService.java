package jp.jaxa.iss.kibo.rpc.tohoku;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */


public class MainService extends KiboRpcService {
    private Wrappers wraps = new Wrappers(api);

    private KiboQRField QRTable[] = {
            new KiboQRField("1-1", 11.5, -5.7, 4.5, 0, 0, 0, 1, QRInfoType.PosX),
            new KiboQRField("1-2", 11, -6, 5.55, 0, -0.7071068, 0, 0.7071068, QRInfoType.PosY),
            new KiboQRField("1-3", 11, -5.5, 4.33, 0, 0.7071068, 0, 0.7071068, QRInfoType.PosZ),
            new KiboQRField("2-1", 10.30, -7.5, 4.7, 0, 0, 1, 0, QRInfoType.QuaX),
            new KiboQRField("2-2", 11.5, -8, 5, 0, 0, 0, 1, QRInfoType.QuaY),
            new KiboQRField("2-3", 11, -7.7, 5.55, 0, -0.7071068, 0, 0.7071068, QRInfoType.QuaZ)
    };

    private KeepZone KZTable[] = {
            new KeepZone(10.75, -4.9, 4.8, 10.95, -4.7, 5.0, false),
            new KeepZone(10.75, -6.5, 3.9, 11.95, -6.4, 5.9, false),
            new KeepZone(9.95, -7.2, 3.9, 10.85, -7.1, 5.9, false),
            new KeepZone(10.10, -8.6, 5.4, 11.1, -83, 5.9, false),
            new KeepZone(11.45, -9.0, 4.1, 11.95, -8.5, 5.1, false),
            new KeepZone(9.95, -9.1, 4.6, 10.45, -8.5, 5.1, false),
            new KeepZone(10.95, -8.4, 4.9, 11.15, -8.2, 5.1, false),
            new KeepZone(11.05, -9.9, 4.2, 11.25, -8.7, 4.4, false),
            new KeepZone(10.45, -9.1, 4.6, 10.65, -8.9, 4.8, false),
            new KeepZone(10.25, -9.75, 4.2, 11.65, -3, 5.6, true),
    };

    private AstrobeeField AstrobeeNode = new AstrobeeField(10.95, -3.75, 4.85, 0, 0, 0.707, -0.707);
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

