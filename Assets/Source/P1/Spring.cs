using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Spring {

    #region InEditorVariables

    public float Stiffness;
    public float Damping;
    public Node nodeA;
    public Node nodeB;

    #endregion

    public enum SpringType { Stretch, Bend };
    public SpringType springType;

    public float Length0;
    public float Length;
    public Vector3 dir;

    private PhysicsManager Manager;

    public Spring(Node a, Node b, SpringType s)
    {
        nodeA = a;
        nodeB = b;
        springType = s;

        dir = nodeA.Pos - nodeB.Pos;
        Length = dir.magnitude;
        Length0 = Length;
    }

    // Use this for initialization
    public void Initialize(float stiffness, float damping, PhysicsManager m)
    {
        Manager = m;
        Stiffness = stiffness;
        Damping = damping;

        //Length0 = Length;
    }

    // Update spring state
    public void UpdateState()
    {
        dir = nodeA.Pos - nodeB.Pos;
        Length = dir.magnitude;
        dir = (1.0f / Length) * dir;
    }

    // Get Force
    public void GetForce(VectorXD force)
    {
        Vector3 temp = (nodeA.Pos - nodeB.Pos) / Length;
        temp.Normalize();
        VectorXD u = DenseVectorXD.OfArray(new double[] { temp.x, temp.y, temp.z });
        float d = Length - Length0;
        temp = nodeA.Vel - nodeB.Vel;
        VectorXD difV = DenseVectorXD.OfArray(new double[] { temp.x, temp.y, temp.z });

        var elasticF = -Stiffness * d * u;
        var dampingF = -Damping * u.ToColumnMatrix() * u.ToRowMatrix() * difV;// * difV.ToRowMatrix();//-Damping * Vector3.Cross(nodeA.Vel - nodeB.Vel, temp);//Stiffness * (nodeA.Vel - nodeB.Vel);
        
        force[nodeA.index] += elasticF[0] + dampingF[0];
        force[nodeA.index + 1] += elasticF[1] + dampingF[1];
        force[nodeA.index + 2] += elasticF[2] + dampingF[2];

        force[nodeB.index] -= elasticF[0] + dampingF[0];
        force[nodeB.index + 1] -= elasticF[1] + dampingF[1];
        force[nodeB.index + 2] -= elasticF[2] + dampingF[2];
    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        //Variables
        Vector3 temp = (nodeA.Pos - nodeB.Pos) / Length;
        temp.Normalize();

        VectorXD u = DenseVectorXD.OfArray(new double[] { temp.x, temp.y, temp.z });
        MatrixXD uT = u.ToRowMatrix();

        float d = (Length - Length0) / Length;

        MatrixXD I = MatrixXD.Build.DenseIdentity(3);

        //Valores Jacobiana
        MatrixXD dFadXa = -Stiffness * d * (I - u.ToColumnMatrix() * u.ToRowMatrix()) - (Stiffness * u.ToColumnMatrix() * u.ToRowMatrix());
        MatrixXD dFbdXb = dFadXa; MatrixXD dFbdXa = -dFadXa; MatrixXD dFadXb = -dFadXa;

        MatrixXD dFadVa = -Damping * u.ToColumnMatrix() * u.ToRowMatrix();
        MatrixXD dFbdVb = dFadVa; MatrixXD dFbdVa = -dFadVa; MatrixXD dFadVb = -dFadVa;

        //Asignacion Valores
        for (int i = 0; i < 3; i++) { 
            for(int j = 0; j < 3; j++)
            {
                //dFdx
                dFdx[nodeA.index + i, nodeA.index + j] += dFadXa[i, j];
                dFdx[nodeB.index + i, nodeB.index + j] += dFbdXb[i, j];
                dFdx[nodeA.index + i, nodeB.index + j] += dFadXb[i, j];
                dFdx[nodeB.index + i, nodeA.index + j] += dFbdXa[i, j];

                //dFdv
                dFdv[nodeA.index + i, nodeA.index + j] += dFadVa[i, j];
                dFdv[nodeB.index + i, nodeB.index + j] += dFbdVb[i, j];
                dFdv[nodeA.index + i, nodeB.index + j] += dFadVb[i, j];
                dFdv[nodeB.index + i, nodeA.index + j] += dFbdVa[i, j];
            }     
        }
    }

}
