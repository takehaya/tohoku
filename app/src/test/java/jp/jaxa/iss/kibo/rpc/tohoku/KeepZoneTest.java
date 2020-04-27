package jp.jaxa.iss.kibo.rpc.tohoku;

import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class KeepZoneTest {

    KeepZone[] KZTable = new KeepZone[2];
    AstrobeeField[] AstrobeeTable = new AstrobeeField[3];

    @Before
    public void setUp() throws Exception {
        KZTable[0] = new KeepZone(10.75, -4.9, 4.8, 10.95, -4.7, 5.0, false);
        KZTable[1] = new KeepZone(10.25, -9.75, 4.2, 11.65, -3, 5.6, true);
        AstrobeeTable[0] = new AstrobeeField(10.95, -3.75, 4.85, 0, 0, 0.707, -0.707); // startline
        AstrobeeTable[1] = new AstrobeeField(10.76, -4.8, 4.9, 0, 0, 0.707, -0.707);
        AstrobeeTable[2] = new AstrobeeField(10.20, -8.75, 4.1, 0, 0, 0.707, -0.707);
    }

    @Test
    public void CheckScope() {
        boolean koz1 = this.KZTable[0].isCheckScope(AstrobeeTable[0]);
        assertEquals("koz testing: Normal scenario", false, koz1);
        boolean kiz1 = this.KZTable[1].isCheckScope(AstrobeeTable[0]);
        assertEquals("kiz testing: Normal scenario", false, kiz1);

        boolean koz2 = this.KZTable[0].isCheckScope(AstrobeeTable[1]);
        assertEquals("koz testing: exception scenario", true, koz2);
        boolean kiz2 = this.KZTable[1].isCheckScope(AstrobeeTable[2]);
        assertEquals("kiz testing: exception scenario", true, kiz2);
    }
}