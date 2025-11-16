package com.kAIS.KAIMyEntity.urdf.control;

import org.joml.Matrix3f;
import org.joml.Vector3f;

/** 3DoF(어깨 yaw(Z), 어깨 pitch(Y), 팔꿈치 flex(Y)) 위치 IK - DLS */
public final class ArmIK3DoF {
    private ArmIK3DoF() {}

    public static class Result { public final float yaw, pitch, elbow;
        public Result(float y, float p, float e){ yaw=y; pitch=p; elbow=e; } }

    public static Result solve(Vector3f shoulderWorldPos, Vector3f targetWorldPos,
                               float[] q0, float L1, float L2, int iters, float lambda) {
        float[] q = { q0!=null && q0.length>=1 ? q0[0] : 0f,
                q0!=null && q0.length>=2 ? q0[1] : 0f,
                q0!=null && q0.length>=3 ? q0[2] : 0f };

        for (int k=0;k<iters;k++){
            Vector3f p = fk(shoulderWorldPos, q, L1, L2);
            Vector3f e = new Vector3f(targetWorldPos).sub(p);
            if (e.lengthSquared() < 1e-6f) break;

            float h=1e-4f; float[][] J = new float[3][3];
            Vector3f pplus, pminus, d;
            for (int i=0;i<3;i++){
                float old=q[i];
                q[i]=old+h; pplus  = fk(shoulderWorldPos, q, L1, L2);
                q[i]=old-h; pminus = fk(shoulderWorldPos, q, L1, L2);
                q[i]=old;
                d = pplus.sub(pminus).mul(1f/(2f*h));
                J[0][i]=d.x; J[1][i]=d.y; J[2][i]=d.z;
            }
            float[][] JT = tr(J); float[][] A = add(mm(JT,J), diag(lambda*lambda));
            float[][] Ainv = inv3(A);
            float[] rhs = mv(JT, e);
            float[] dq  = mv(Ainv, rhs);
            q[0]+=dq[0]; q[1]+=dq[1]; q[2]+=dq[2];
        }
        return new Result(q[0], q[1], q[2]);
    }

    /** FK: R = Rz(yaw)*Ry(pitch), elbow에서 Ry(elbow) 추가 */
    private static Vector3f fk(Vector3f sh, float[] q, float L1, float L2){
        float yaw=q[0], pitch=q[1], elbow=q[2];
        Matrix3f R = new Matrix3f().rotationZ(yaw).rotateY(pitch);
        Vector3f elbowPos = new Vector3f(L1,0,0); R.transform(elbowPos).add(sh);
        Matrix3f Re = new Matrix3f(R).rotateY(elbow);
        Vector3f wristOff = new Vector3f(L2,0,0); Re.transform(wristOff);
        return elbowPos.add(wristOff);
    }

    // --- small 3x3 helpers ---
    private static float[][] tr(float[][] M){ return new float[][]{
            {M[0][0],M[1][0],M[2][0]},{M[0][1],M[1][1],M[2][1]},{M[0][2],M[1][2],M[2][2]} }; }
    private static float[][] mm(float[][] A,float[][] B){ float[][] C=new float[3][3];
        for(int i=0;i<3;i++) for(int j=0;j<3;j++)
            C[i][j]=A[i][0]*B[0][j]+A[i][1]*B[1][j]+A[i][2]*B[2][j]; return C; }
    private static float[] mv(float[][] A, Vector3f v){ return new float[]{
            A[0][0]*v.x+A[0][1]*v.y+A[0][2]*v.z,
            A[1][0]*v.x+A[1][1]*v.y+A[1][2]*v.z,
            A[2][0]*v.x+A[2][1]*v.y+A[2][2]*v.z }; }
    private static float[] mv(float[][] A, float[] x){ return new float[]{
            A[0][0]*x[0]+A[0][1]*x[1]+A[0][2]*x[2],
            A[1][0]*x[0]+A[1][1]*x[1]+A[1][2]*x[2],
            A[2][0]*x[0]+A[2][1]*x[1]+A[2][2]*x[2] }; }
    private static float[][] add(float[][] A,float[][] B){ float[][] C=new float[3][3];
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) C[i][j]=A[i][j]+B[i][j]; return C; }
    private static float[][] diag(float d){ return new float[][]{{d,0,0},{0,d,0},{0,0,d}}; }
    private static float[][] inv3(float[][] a){ float a00=a[0][0],a01=a[0][1],a02=a[0][2];
        float a10=a[1][0],a11=a[1][1],a12=a[1][2];
        float a20=a[2][0],a21=a[2][1],a22=a[2][2];
        float det = a00*(a11*a22-a12*a21)-a01*(a10*a22-a12*a20)+a02*(a10*a21-a11*a20);
        float id=1f/det;
        return new float[][]{
                {(a11*a22-a12*a21)*id,(a02*a21-a01*a22)*id,(a01*a12-a02*a11)*id},
                {(a12*a20-a10*a22)*id,(a00*a22-a02*a20)*id,(a02*a10-a00*a12)*id},
                {(a10*a21-a11*a20)*id,(a01*a20-a00*a21)*id,(a00*a11-a01*a10)*id} };
    }
}
