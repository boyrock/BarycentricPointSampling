using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class PointSampler : MonoBehaviour
{
    Mesh mesh;

    Triangle[] triangles;

    Vector3[] meshVertics;
    int[] meshTriangles;

    List<Vector3> positions = new List<Vector3>();

    [SerializeField]
    int count = 1000;

    [SerializeField]
    GameObject targetObject;

    [SerializeField]
    float weightPower;

    [SerializeField]
    VertexWeightType vertexWeightType;
    IComputableVertexWeight vertexWeightComputer;

    private void Awake()
    {
        Init();
    }

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < count; i++)
        {
            Sampling();
        }
    }

    private void Sampling()
    {
        var rand = UnityEngine.Random.value;

        var triangle = BisectionSearchTriangle(rand);
        if (triangle == null)
            return;

        var u = U(triangle);
        var v = V(triangle, u);
        var w = W(u, v);
        var p = triangle.v0.pos * u + triangle.v1.pos * v + triangle.v2.pos * w;

        positions.Add(p);
    }

    private void Init()
    {
        var meshFilter = targetObject.GetComponent<MeshFilter>();
        if (meshFilter != null)
            mesh = meshFilter.mesh;

        var skinMesh = targetObject.GetComponent<SkinnedMeshRenderer>();
        if (skinMesh != null)
            mesh = skinMesh.sharedMesh;

        meshVertics = mesh.vertices;
        meshTriangles = mesh.triangles;


        switch (vertexWeightType)
        {
            case VertexWeightType.AreaDensity:
                vertexWeightComputer = new AreaDensityVertexWeight();
                break;
            case VertexWeightType.NeighborhoodDensity:
                vertexWeightComputer = new NeighborhoodDensityVertexWeight();
                break;
            case VertexWeightType.Curvature:
                vertexWeightComputer = new CurvatureVertexWeight();
                break;
            default:
                break;
        }
        float[] weights = vertexWeightComputer.ComputeWeight(targetObject.transform,meshVertics, meshTriangles, weightPower);
        
        float maxWeight = weights.Max();

        Color[] colors = new Color[meshVertics.Length];

        for (int i = 0; i < meshVertics.Length; i++)
        {
            float t = weights[i] / maxWeight;
            colors[i] = Color.Lerp(Color.blue, Color.red, t);
        }

        mesh.colors = colors;

        triangles = new Triangle[meshTriangles.Length / 3];

        int ii = 0;
        for (int i = 0; i < meshTriangles.Length; i += 3)
        {
            int i0 = meshTriangles[i];
            int i1 = meshTriangles[i + 1];
            int i2 = meshTriangles[i + 2];

            Vertex v0 = new Vertex(i0, targetObject.transform.TransformPoint(meshVertics[i0]));
            Vertex v1 = new Vertex(i1, targetObject.transform.TransformPoint(meshVertics[i1]));
            Vertex v2 = new Vertex(i2, targetObject.transform.TransformPoint(meshVertics[i2]));

            Triangle triangle = new Triangle(v0, v1, v2);

            triangle.v0.SetWeight(weights[i0] / maxWeight);
            triangle.v1.SetWeight(weights[i1] / maxWeight);
            triangle.v2.SetWeight(weights[i2] / maxWeight);

            triangles[ii] = triangle;

            ii++;
        }

        var totalWeightedArea = triangles.Sum(t => t != null ? t.GetWeightedArea() : 0);
        float sumWeightedArea = 0f;

        for (int i = 0; i < triangles.Length; i++)
        {
            var triangle = triangles[i];
            sumWeightedArea += triangle.GetWeightedArea();
            var cdf = sumWeightedArea / totalWeightedArea;
            triangle.cdf = cdf;
        }
    }

    private Triangle BisectionSearchTriangle(float rand)
    {
        int left = 0;
        int right = triangles.Length - 1;

        while (left <= right)
        {
            int mid = (left + right) / 2;

            var minIndex = mid - 1 < 0 ? 0 : mid - 1;
            var maxIndex = mid;
            var min = triangles[minIndex].cdf;
            var max = triangles[maxIndex].cdf;

            if (rand >= min && rand < max)
            {
                return triangles[mid];
            }
            else if (triangles[mid].cdf < rand)
                left = mid + 1;
            else
                right = mid - 1;
        }

        return null;
    }

    private float U(Triangle triangle, int maxIterations = 20)
    {
        float phi_u = triangle.phi_u;
        float phi_v = triangle.phi_v;

        float r = UnityEngine.Random.value;
        float l = (2.0f * phi_u - phi_v) / 3.0f;
        float u = 0.5f;
        float n = 0;

        while (n++ < maxIterations)
        {
            float u1 = 1.0f - u;
            float p = u * (2.0f - u) - l * u * u1 * u1 - r;
            float pd = Mathf.Max(u1 * (2.0f + l * (3.0f * u - 1.0f)), Mathf.Epsilon);
            float du = Mathf.Max(Mathf.Min(p / pd, 0.25f), -0.25f);
            u -= du;
            u = Mathf.Max(Mathf.Min(u, 1.0f-Mathf.Epsilon), Mathf.Epsilon);
            if (Mathf.Abs(du) < Mathf.Epsilon)
                break;
        }

        return u;
    }

    private float V(Triangle triangle, float u)
    {
        float r = UnityEngine.Random.value;

        if (Mathf.Abs(triangle.phi_v) < Mathf.Epsilon)
        {
            return (1.0f - u) * r;
        }

        float tau = Tau(triangle, u);

        float tmp = tau + u - 1.0f;

        //v± = τ ± sqrt(τ²(1−ξv)+(τ+u−1)²ξv) 
        float q = Mathf.Sqrt(tau * tau * (1.0f - r) + tmp * tmp * r);

        return tau <= 0.5f * (1.0f - u) ? tau + q : tau - q;
    }
    private float W(float u, float v)
    {
        return 1.0f - u - v;
    }

    private float Tau(Triangle triangle, float u)
    {
        //tau = 1.0/3.0 - (1.0 + (u-1.0/3.0)*Phi_u)/Phi_v;
        float dived = 1.0f / 3.0f;
        return dived - (1.0f + (u - dived) * triangle.phi_u) / triangle.phi_v;
    }

    [SerializeField]
    float gizmoSize = 0.005f;

    private void OnDrawGizmos()
    {
        if (positions.Count == 0)
            return;

        Gizmos.color = Color.red;
        foreach (var pos in positions)
        {
            Gizmos.DrawSphere(pos, gizmoSize);
        }
    }
}

public class Vertex
{
    public int index;
    public Vector3 pos;
    public float weight { get; protected set; }

    public Vertex() { }

    public Vertex(Vector3 pos)
    {
        this.pos = pos;
    }
    public Vertex(int index, Vector3 pos)
    {
        this.index = index;
        this.pos = pos;
    }
    public void SetWeight(float w)
    {
        weight = w;
    }
}

public class Triangle
{
    public int index;

    public Vertex v0;
    public Vertex v1;
    public Vertex v2;

    public float cdf;

    public float pdf;

    public float phi_u
    {
        get
        {
            float phi_u = (ComputeBarycentricCoordinatesWeight(v0.pos) - ComputeBarycentricCoordinatesWeight(v2.pos)) / avgWeight;

            return phi_u;
        }
    }
    public float phi_v
    {
        get
        {

            float phi_v = (ComputeBarycentricCoordinatesWeight(v1.pos) - ComputeBarycentricCoordinatesWeight(v2.pos)) / avgWeight;

            return phi_u;
        }
    }

    public float avgWeight
    {
        get
        {
            float avgWeight = (ComputeBarycentricCoordinatesWeight(v0.pos) +
                          ComputeBarycentricCoordinatesWeight(v1.pos) +
                          ComputeBarycentricCoordinatesWeight(v2.pos)) / 3.0f;

            return avgWeight;
        }
    }

    public Triangle(Vertex v0, Vertex v1, Vertex v2)
    {
        this.v0 = v0;
        this.v1 = v1;
        this.v2 = v2;
    }

    public float GetArea()
    {
        return Vector3.Cross(v1.pos - v0.pos, v2.pos - v0.pos).magnitude * 0.5f;
    }
    private float GetWeightAverage()
    {
        return (v0.weight + v1.weight + v2.weight) / 3f;
    }

    public float GetWeightedArea()
    {
        return GetArea() * GetWeightAverage();
    }
    
    Vector3 ComputeBarycentricCoordinates(Vector3 x)
    {
        Vector3 p0 = v1.pos - v0.pos;
        Vector3 p1 = v2.pos - v0.pos;
        Vector3 p2 = x - v0.pos;

        float d00 = Vector3.Dot(p0, p0);
        float d01 = Vector3.Dot(p0, p1);
        float d11 = Vector3.Dot(p1, p1);
        float d20 = Vector3.Dot(p2, p0);
        float d21 = Vector3.Dot(p2, p1);

        float denom = d00 * d11 - d01 * d01;

        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;

        return new Vector3(u, v, w);
    }

    public float ComputeBarycentricCoordinatesWeight(Vector3 x)
    {
        Vector3 barycentric = ComputeBarycentricCoordinates(x);

        //φ(x) = u(x)(φu−φw) + v(x)(φv−φw) + φw
        return barycentric.x * (v0.weight - v2.weight) + 
               barycentric.y * (v1.weight - v2.weight) + 
               v2.weight;
    }
}

public enum VertexWeightType
{
    AreaDensity,
    NeighborhoodDensity,
    Curvature
}

