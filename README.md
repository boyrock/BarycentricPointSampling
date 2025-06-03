# BarycentricPointSampling

**BarycentricPointSampling** is a Python-based tool for uniformly sampling points on the surface of a 3D mesh using barycentric coordinates. It generates point clouds from triangle meshes with probability proportional to triangle area.

## Features

- Surface point sampling using barycentric coordinates
- Triangle area-based importance sampling
- High-performance vectorized implementation using NumPy
- Easy to integrate into existing mesh processing pipelines

## How It Works
- Compute the area of each triangle in the mesh.
- Use those areas to build a probability distribution function (PDF) and cumulative distribution function (CDF).
- Sample triangles using the CDF (inversion sampling).

## Reference
[Efficient barycentric point sampling on meshes](https://arxiv.org/pdf/1708.07559)
