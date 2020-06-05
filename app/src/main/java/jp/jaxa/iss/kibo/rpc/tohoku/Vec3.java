package jp.jaxa.iss.kibo.rpc.tohoku;

import android.util.Log;

import gov.nasa.arc.astrobee.types.*;

public class Vec3 extends Point {

    public Vec3() {
        setXYZ(0,0,0);

    }

    public Vec3(double x){
        setXYZ(x,x,x);
    }

    public Vec3(double x, double y, double z) {
        setXYZ(x,y,z);
    }

    public void setX(double x) {
        m_vec[0] = x;
    }

    public void setY(double y) {
        m_vec[1] = y;
    }

    public void setZ(double z) {
        m_vec[2] = z;
    }

    public void setXYZ(double x,double y,double z){
        setX(x);
        setY(y);
        setZ(z);
    }

    @Override
    public String toString() {
        return "{ "+getX()+", "+getY()+", "+getZ()+" }";
    }

    @Override
    public boolean equals(Object obj) {
        Vec3 vec3=(Vec3) obj;
        return obj.getClass() == this.getClass() && vec3.getX() == getX() && vec3.getY() == getY() && vec3.getZ() == getZ();
    }

    public Vec3 toMinus(){
        return new Vec3(-getX(),-getY(),-getZ());
    }

    public Vec3 add(Vec3 v) {
        return new Vec3(getX() + v.getX(),getY() + v.getY(),getZ() + v.getZ());
    }

    public Vec3 sub(Vec3 v){
        return add(v.toMinus());
    }

    public Vec3 mul(double x) {
        return new Vec3(getX() * x,getY() * x,getY() * x);
    }

    public double dot(Vec3 v) {
        return getX() * v.getX() + getY() * v.getY() + getZ() * v.getZ();
    }

    public double dotnormal(Vec3 v) {
        double d = dot(v);
        double s1 = Math.sqrt(getX()*getX() + getY()*getY() + getZ()*getZ());
        double s2 = Math.sqrt(v.getX()*v.getX() + v.getY()+v.getY() + v.getZ()*v.getZ());
        return (d/(s1*s2));
    }

    public double norm3(){
        return Math.sqrt(getX()*getX() + getY() * getY() + getZ() * getZ());
    }

    public static double norm2(double a1, double a2){
        return Math.sqrt(a1*a1 + a2*a2);
    }

    public static double dot2vecNormal(double a1, double a2, double b1, double b2) {
        Log.d(MainService.LOGTAG, "dot2vecNormal start");

        // normalize
        double maga = 1/Math.sqrt(a1*a1 + a2*a2);
        double magb = 1/Math.sqrt(b1*b1 + b2*b2);
        a1 *= maga;
        a2 *= maga;
        b1 *= magb;
        b2 *= magb;

        double d = a1*b1 + a2*b2;
        Log.d(MainService.LOGTAG, "dot2vecNormal d: " + d);

        double s1 = Math.sqrt(a1*a1 + a2*a2);
        Log.d(MainService.LOGTAG, "dot2vecNormal s1: " + s1);

        double s2 = Math.sqrt(b1*b1 + b2*b2);
        Log.d(MainService.LOGTAG, "dot2vecNormal s2: " + s2);

        double r1 = d/(s1*s2);
        Log.d(MainService.LOGTAG, "dot2vecNormal r1: " + r1);

        return r1;
    }


    public Vec3 cross(Vec3 v) {
        return new Vec3(
                getY() * v.getZ() - getZ() * v.getY(),
                -getX() * v.getZ() + getZ() * v.getX(),
                getX() * v.getY() - getY() * v.getX()
        );
    }

    public double length() {
        return Math.sqrt(this.dot(this));
    }

    public Vec3 normalization() {
        return this.mul(1.0 / length());
    }

}
