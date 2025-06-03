using UnityEngine;

public interface IComputableVertexWeight
{
    public abstract float[] ComputeWeight(Transform transform, Vector3[] vertics, int[] triangles, float weightPower);
}