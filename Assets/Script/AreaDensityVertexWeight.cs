using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class AreaDensityVertexWeight : IComputableVertexWeight
{
    public float[] ComputeWeight(Transform transform, Vector3[] vertices, int[] triangles, float weightPower)
    {

        float[] weights = new float[vertices.Length];

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int i0 = triangles[i];
            int i1 = triangles[i + 1];
            int i2 = triangles[i + 2];

            Vector3 p0 = transform.TransformPoint(vertices[i0]);
            Vector3 p1 = transform.TransformPoint(vertices[i1]);
            Vector3 p2 = transform.TransformPoint(vertices[i2]);

            float area = Vector3.Cross(p1 - p0, p2 - p0).magnitude * 0.5f;

            weights[i0] += area;
            weights[i1] += area;
            weights[i2] += area;
        }

        Dictionary<Vector3, List<int>> vertexGroups = new();

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 pos = vertices[i];
            if (!vertexGroups.ContainsKey(pos))
                vertexGroups[pos] = new List<int>();

            vertexGroups[pos].Add(i);
        }

        float[] mergedWeights = new float[vertices.Length];
        foreach (var kvp in vertexGroups)
        {
            float sum = kvp.Value.Select(i => weights[i]).Sum();
            foreach (int i in kvp.Value)
                mergedWeights[i] = Mathf.Pow(1.0f / sum, weightPower);
        }

        return mergedWeights;
    }
}

