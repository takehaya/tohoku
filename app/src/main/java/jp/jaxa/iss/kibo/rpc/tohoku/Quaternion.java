package jp.jaxa.iss.kibo.rpc.tohoku;

import gov.nasa.arc.astrobee.types.Quaternion;

public class WrapQuaternion extends Quaternion {
    private static Quaternion  identityQuaternion = new Quaternion();
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
    public static Quaternion getIdentity(){
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
    public static Quaternion mul(Quaternion lhs, Quaternion rhs)
    {
        return new Quaternion(
                lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
                lhs.w * rhs.y + lhs.y * rhs.w + lhs.z * rhs.x - lhs.x * rhs.z,
                lhs.w * rhs.z + lhs.z * rhs.w + lhs.x * rhs.y - lhs.y * rhs.x,
                lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z
        );
    }
    // The dot product between two rotations.
    public static float dot(Quaternion a, Quaternion b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }
}
