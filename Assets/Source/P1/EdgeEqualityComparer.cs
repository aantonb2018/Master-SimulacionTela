using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Edge
{
    public int A; public int B; public int O;

    public Edge(int a, int b, int o)
    {
        A = a;
        B = b;
        O = o;
    }
}

public class EdgeEqualityComparer : IEqualityComparer<Edge>
{

    public bool Equals(Edge a, Edge b)
    {
        //Si a y b son iguales
        if(a.A == b.A && a.B == b.B)
            return true;
        else//Si a y b no son iguales
            return false;
    }

    public int GetHashCode(Edge e)
    {
        int hcode = 17;
        hcode = hcode * 23 + e.A.GetHashCode();
        hcode = hcode * 23 + e.B.GetHashCode();
        hcode = hcode * 23 + e.O.GetHashCode();

        return hcode;
    }
}