using UnityEngine;

public class NeighborhoodDensityVertexWeight : IComputableVertexWeight
{
    public float[] ComputeWeight(Transform transform, Vector3[] vertices, int[] triangles, float weightPower)
    {
        float[] density = new float[vertices.Length];
        
        for (int i = 0; i < vertices.Length; i++)
        {
            var vi = transform.TransformPoint(vertices[i]);
            float sumDistance = 0;
            int neighborCount = 0;

            for (int j = 0; j < vertices.Length; j++)
            {
                if (i == j)
                    continue;

                float dist = Vector3.Distance(vi, transform.TransformPoint(vertices[j]));

                if (dist < transform.transform.localScale.x)
                {
                    sumDistance += dist;
                    neighborCount++;
                }
            }

            if (neighborCount > 0)
            {
                float avgDist = sumDistance / neighborCount;

                avgDist = Mathf.Pow(avgDist, weightPower);
                density[i] = 1f / avgDist;
            }
            else
            {
                density[i] = 0f;
            }
        }


        return density;
    }
}
