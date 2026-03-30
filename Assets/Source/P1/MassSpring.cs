using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
using MathNet.Numerics.Distributions;
using UnityEngine.UIElements;
using System;

/// <summary>
/// Basic mass-spring model component which can be dropped onto
/// a game object and configured so that the set of nodes and
/// edges behave as a mass-spring model.
/// </summary>
public class MassSpring : MonoBehaviour, ISimulable
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public MassSpring()
    {
        Manager = null;
    }

    #region EditorVariables

    public List<Node> Nodes;
    public List<Spring> Springs;

    public float Mass;
    public float StiffnessStretch;
    public float StiffnessBend;
    public float DampingAlpha;
    public float DampingBeta;

    #endregion

    #region OtherVariables
    private PhysicsManager Manager;

    private int index;
    #endregion

    #region MonoBehaviour

    public void Awake()
    {
        Mesh mesh = this.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        Nodes = new List<Node>();
        Springs = new List<Spring>();
        //Nodos
        foreach (Vector3 v in vertices) {
            Node n = new Node(transform.TransformPoint(v));
            Nodes.Add(n);
        }

        InitSprings(triangles);
        /*
        float numTriangles = triangles.Length / 3;
        for (int i = 0; i <= numTriangles - 1; i++)
        {
            int j = i * 3;

            //Debug.Log(j + " " + i + " " +numTriangles);
            int vertex1 = triangles[j];
            int vertex2 = triangles[j + 1];
            int vertex3 = triangles[j + 2];

            Edge newEdge1 = new Edge(vertex1, vertex2, vertex3);
            Edges.Add(newEdge1);
            Edge newEdge2 = new Edge(vertex2, vertex3, vertex1);
            Edges.Add(newEdge2);
            Edge newEdge3 = new Edge(vertex3, vertex1, vertex2);
            Edges.Add(newEdge3);
        }

        Edges.Sort();

        //Springs
        Spring nextSpring;
        nextSpring = new Spring(Nodes[Edges[0].vertexA], Nodes[Edges[0].vertexB], Spring.SpringType.Stretch);
        Springs.Add(nextSpring);

        for (int i = 1; i < Edges.Count; i++)
        {
            if (Edges[i].Compare(Edges[i]))
            {
                nextSpring = new Spring(Nodes[Edges[i].vertexOther], Nodes[Edges[i - 1].vertexOther], Spring.SpringType.Bend);

                Springs.Add(nextSpring);
            }
            else
            {
                nextSpring = new Spring(Nodes[Edges[i].vertexA], Nodes[Edges[i].vertexB], Spring.SpringType.Stretch);
                Springs.Add(nextSpring);
            }
        }*/
    }
    
    private void InitSprings(int[] triangles)
    {
        EdgeEqualityComparer edgeComparer = new EdgeEqualityComparer();
        Dictionary<Edge, Edge> edgeDictionary = new Dictionary<Edge, Edge>(edgeComparer);
        //SET UP SPRINGS
        for (int index = 0; index < triangles.Length; index += 3)
        {
            List<Edge> edges = new List<Edge>();
            edges.Add(new Edge(triangles[index], triangles[index + 1], triangles[index + 2]));
            edges.Add(new Edge(triangles[index + 1], triangles[index + 2], triangles[index]));
            edges.Add(new Edge(triangles[index], triangles[index + 2], triangles[index + 1]));

            foreach (Edge edge in edges)
            {
                Edge auxEdge;
                if (edgeDictionary.TryGetValue(edge, out auxEdge))
                {
                    auxEdge.A = auxEdge.O;
                    auxEdge.B = edge.O;
                    Spring spring = new Spring(Nodes[auxEdge.A], Nodes[auxEdge.B], Spring.SpringType.Bend);
                    spring.Initialize(StiffnessBend, DampingBeta, Manager);
                    Springs.Add(spring);
                }
                else
                {
                    Spring spring = new Spring(Nodes[edge.A], Nodes[edge.B], Spring.SpringType.Stretch);
                    spring.Initialize(StiffnessStretch, DampingBeta, Manager);
                    Springs.Add(spring);
                    edgeDictionary.Add(edge, edge);
                }
            }
        }
    }

    public void Update()
    {
        // TO BE COMPLETED
    }

    public void FixedUpdate()
    {
        Mesh mesh = this.GetComponent<MeshFilter>().mesh;
        List<Vector3> vertices = new List<Vector3>();  
        for (int i = 0;i < Nodes.Count; i++)
        {
            
            vertices.Add(transform.InverseTransformPoint(Nodes[i].Pos));
        }
        mesh.SetVertices(vertices);
        // TO BE COMPLETED
    }
    #endregion

    #region ISimulable

    public void Initialize(int ind, PhysicsManager m, List<Fixer> fixers)
    {
        Manager = m;

        index = ind;

        // Start scene nodes/edges
        for (int i = 0; i < Nodes.Count; ++i)
        {
            Nodes[i].Initialize(index + 3 * i, Mass/Nodes.Count, (Mass/Nodes.Count) * DampingAlpha, Manager); // Prepare
            foreach(Fixer f in fixers)
            {
                if (f.IsInside(Nodes[i].Pos))
                {
                    Nodes[i].Fixed = true;
                }
            }
        }


        for (int i = 0; i < Springs.Count; ++i)
        {
            switch (Springs[i].springType)
            {
                case Spring.SpringType.Stretch:
                    Springs[i].Initialize(StiffnessStretch, StiffnessStretch * DampingBeta, Manager); // Prepare
                    break;
                case Spring.SpringType.Bend:
                    Springs[i].Initialize(StiffnessBend, StiffnessBend * DampingBeta, Manager); // Prepare
                    break;
            }
        }
            
    }

    public int GetNumDoFs()
    {
        return 3 * Nodes.Count;
    }

    public void GetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetPosition(position);
    }

    public void SetPosition(VectorXD position)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetPosition(position);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].UpdateState();
    }

    public void GetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetVelocity(velocity);
    }

    public void SetVelocity(VectorXD velocity)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].SetVelocity(velocity);
    }

    public void GetForce(VectorXD force)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForce(force);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].GetForce(force);
    }

    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetForceJacobian(dFdx, dFdv);
        for (int i = 0; i < Springs.Count; ++i)
            Springs[i].GetForceJacobian(dFdx, dFdv);
    }

    public void GetMass(MatrixXD mass)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMass(mass);
    }

    public void GetMassInverse(MatrixXD massInv)
    {
        for (int i = 0; i < Nodes.Count; ++i)
            Nodes[i].GetMassInverse(massInv);
    }

    public void FixVector(VectorXD v)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixVector(v);
        }
    }

    public void FixMatrix(MatrixXD M)
    {
        for (int i = 0; i < Nodes.Count; i++)
        {
            Nodes[i].FixMatrix(M);
        }
    }

    #endregion

    #region OtherMethods

    #endregion

}
