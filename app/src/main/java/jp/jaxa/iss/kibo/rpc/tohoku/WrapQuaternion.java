package jp.jaxa.iss.kibo.rpc.tohoku;

import android.util.Log;

import gov.nasa.arc.astrobee.types.Quaternion;

public class WrapQuaternion extends Quaternion {
    private static WrapQuaternion  identityQuaternion = new WrapQuaternion();
    public final static double kEpsilon = 0.000001F;
    public WrapQuaternion() {
        this(0.0F, 0.0F, 0.0F, 1.0F);
    }
    public WrapQuaternion(float x, float y, float z, float w) {
        super(x, y, z, w);
    }

    public void SetXYZW(float newX, float newY, float newZ, float newW) {
        setX(newX);
        setY(newY);
        setZ(newZ);
        setW(newW);
    }
    public void setX(float x) { this.m_vec[0] = x; }

    public void setY(float y) {
        this.m_vec[1] = y;
    }

    public void setZ(float z) { this.m_vec[2] = z; }

    public void setW(float w) { this.m_vec[3] = w; }

    // The identity rotation (RO). This quaternion corresponds to "no rotation": the object
    public static WrapQuaternion getIdentity(){
            return identityQuaternion;
    }

    //usage:
    //IsEqualUsingDot(Dot(lhs, rhs));
    public static Boolean equals(float dot) {
        // Returns false in the presence of NaN values.
        return dot > 1.0f - kEpsilon;
    }

    public static Boolean notEqual(Quaternion lhs, Quaternion rhs)
    {
        // Returns true in the presence of NaN values.
        return !(lhs == rhs);
    }

    // Combines rotations /lhs/ and /rhs/.
    public static WrapQuaternion mul(Quaternion lhs, Quaternion rhs)
    {
        return new WrapQuaternion(
                lhs.getW() * rhs.getX() + lhs.getX() * rhs.getW() + lhs.getY() * rhs.getZ() - lhs.getZ() * rhs.getY(),
                lhs.getW() * rhs.getY() + lhs.getY() * rhs.getW() + lhs.getZ() * rhs.getX() - lhs.getX() * rhs.getZ(),
                lhs.getW() * rhs.getZ() + lhs.getZ() * rhs.getW() + lhs.getX() * rhs.getY() - lhs.getY() * rhs.getX(),
                lhs.getW() * rhs.getW() - lhs.getX() * rhs.getX() - lhs.getY() * rhs.getY() - lhs.getZ() * rhs.getZ()
        );
    }

    public static WrapQuaternion Inverse(WrapQuaternion quaternion)
    {
        double num2 = (((quaternion.getX() * quaternion.getX()) + (quaternion.getY() * quaternion.getY())) + (quaternion.getZ() * quaternion.getZ())) + (quaternion.getW() * quaternion.getW());
        double num = 1.0 / num2;

        WrapQuaternion quaternion2 = new WrapQuaternion(
                (float) (-1 * quaternion.getX() * num),
                (float) (-1 * quaternion.getY() * num),
                (float) (-1 * quaternion.getZ() * num),
                (float) (-1 * quaternion.getW() * num)
        );
        return quaternion2;
    }

    // The dot product between two rotations.
    public static float dot(WrapQuaternion a, WrapQuaternion b)
    {
        return a.getX() * b.getX() + a.getY() * b.getY() + a.getZ() * b.getZ() + a.getW() * b.getW();
    }

    public static Vec3 vec3mul(WrapQuaternion rotation, Vec3 point)
    {
        float x = rotation.getX() * 2F;
        float y = rotation.getY() * 2F;
        float z = rotation.getZ() * 2F;
        float xx = rotation.getX() * x;
        float yy = rotation.getY() * y;
        float zz = rotation.getZ() * z;
        float xy = rotation.getX() * y;
        float xz = rotation.getX() * z;
        float yz = rotation.getY() * z;
        float wx = rotation.getW() * x;
        float wy = rotation.getW() * y;
        float wz = rotation.getW() * z;

        double vx = (1F - (yy + zz)) * point.getX() + (xy - wz) * point.getY() + (xz + wy) * point.getZ();
        double vy = (xy + wz) * point.getX() + (1F - (xx + zz)) * point.getY() + (yz - wx) * point.getZ();
        double vz = (xz - wy) * point.getX() + (yz + wx) * point.getY() + (1F - (xx + yy)) * point.getZ();
        return new Vec3(vx, vy, vz);
    }

    public static WrapQuaternion EulerZYX(Vec3 xyz){
        double t0 = Math.cos(xyz.getX()*0.5);
        double t1 = Math.sin(xyz.getX()*0.5);
        double t2 = Math.cos(xyz.getY()*0.5);
        double t3 = Math.cos(xyz.getY()*0.5);
        double t4 = Math.cos(xyz.getZ()*0.5);
        double t5 = Math.cos(xyz.getZ()*0.5);

        WrapQuaternion Q = new WrapQuaternion();
        Q.setX((float)(t0 * t2 * t4 + t1 * t3 * t5));
        Q.setY((float)(t0 * t3 * t4 - t1 * t2 * t5));
        Q.setZ((float)(t0 * t2 * t5 + t1 * t3 * t4));
        Q.setW((float)(t1 * t2 * t4 - t0 * t3 * t5));
        Log.d(MainService.LOGTAG, "EulerZYX Q: "+ Q.toString());

        return Q;
    }
}
