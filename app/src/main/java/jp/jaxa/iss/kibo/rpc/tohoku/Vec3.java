package jp.jaxa.iss.kibo.rpc.tohoku;

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

    public Vec3 add(Vec3 v) {
        setX(getX() + v.getX());
        setY(getY() + v.getY());
        setZ(getZ() + v.getZ());
        return this;
    }

    public Vec3 mul(double x) {
        setX(getX() * x);
        setY(getY() * x);
        setZ(getZ() * x);
        return this;
    }

    public double dot(Vec3 v) {
        return getX() * v.getX() + getY() * v.getY() + getZ() * v.getZ();
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
