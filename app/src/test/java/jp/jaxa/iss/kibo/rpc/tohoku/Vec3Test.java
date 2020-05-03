package jp.jaxa.iss.kibo.rpc.tohoku;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class Vec3Test {

    Vec3 v1=new Vec3(2,2,2);
    Vec3 v2=new Vec3(-1,-1,-1);
    Vec3 v3=new Vec3(2,-2,2);

    @Before
    public void setUp() throws Exception {
    }

    @Test
    public void add() {
        assertEquals(v1.add(v2).getX(),new Vec3(1,1,1).getX(),0.1);
    }

    @Test
    public void mul() {
        assertEquals(v1.mul(2).getX(),new Vec3(4,4,4).getX(),0.1);
    }

    @Test
    public void dot() {
        assertEquals(v1.dot(v2),-6,0.1);
    }

    @Test
    public void cross() {
        assertEquals(v1.cross(v3).getX(),new Vec3(8,0,-8).getX(),0.1);
    }

    @Test
    public void length() {
        assertEquals(v1.length(),Math.sqrt(12),0.1);
    }

    @Test
    public void normalization() {

        assertEquals(v1.normalization().length(),1,0.1);
    }
}
