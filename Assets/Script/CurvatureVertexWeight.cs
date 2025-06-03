using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class CurvatureVertexWeight : IComputableVertexWeight
{
    public float[] ComputeWeight(Transform transform, Vector3[] vertices, int[] triangles, float weightPower)
    {
        Dictionary<int, List<Vector3>> vertexNormals = new();
        
        double[] sumOfAnglesAroundVertex = new double[vertices.Length];

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int i0 = triangles[i];
            int i1 = triangles[i + 1];
            int i2 = triangles[i + 2];

            Vector3 p0 = transform.TransformPoint(vertices[i0]);
            Vector3 p1 = transform.TransformPoint(vertices[i1]);
            Vector3 p2 = transform.TransformPoint(vertices[i2]);

            Vector3 edge01 = p1 - p0;
            Vector3 edge02 = p2 - p0;
            if (edge01.sqrMagnitude > Mathf.Epsilon && edge02.sqrMagnitude > Mathf.Epsilon)
            {
                sumOfAnglesAroundVertex[i0] += Vector3.Angle(edge01, edge02) * Mathf.Deg2Rad;
            }

            Vector3 edge10 = p0 - p1;
            Vector3 edge12 = p2 - p1;
            if (edge10.sqrMagnitude > Mathf.Epsilon && edge12.sqrMagnitude > Mathf.Epsilon)
            {
                sumOfAnglesAroundVertex[i1] += Vector3.Angle(edge10, edge12) * Mathf.Deg2Rad;
            }

            Vector3 edge20 = p0 - p2;
            Vector3 edge21 = p1 - p2;
            if (edge20.sqrMagnitude > Mathf.Epsilon && edge21.sqrMagnitude > Mathf.Epsilon)
            {
                sumOfAnglesAroundVertex[i2] += Vector3.Angle(edge20, edge21) * Mathf.Deg2Rad;
            }
        }

        float[] curvatureValues = new float[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            curvatureValues[i] = (float)((2.0 * Mathf.PI) - sumOfAnglesAroundVertex[i]);
        }

        return curvatureValues;
    }
}
