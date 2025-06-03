# BarycentricPointSampling
This algorithm is designed for the unbiased random sampling of points on a triangle mesh. 
The density of these sampled points on the surface is determined by barycentric interpolation of non-negative per-vertex weights. 
The algorithm's correctness has been verified using statistical tests. 

## Features
- It provides an easy-to-implement and efficient analytical inversion method for point sampling.
- It generates statistically unbiased random samples across the mesh surface.
- The sampling density is controlled by barycentrically interpolating non-negative per-vertex weights.
- It offers an analytical solution, avoiding the need for complex preprocessing or general rejection sampling for density within a triangle.
- It is faster on average than rejection sampling when sampling points within a single triangle.
- Its correctness has been statistically verified.

## How It Works
- Triangle Selection: A triangle is chosen with a probability proportional to its area multiplied by its average weight. This is done using the cumulative distribution function (CDF) of the triangles.
- Point Sampling within the Chosen Triangle: Within the selected triangle, the point's location is determined by sampling its barycentric coordinates (u, v) using analytical inversion
- The u coordinate is sampled by solving the equation PU(u) = ξu, where PU(u) is the marginal CDF of u (a cubic polynomial), and ξu is a uniform random number. This cubic equation is solved efficiently using Newton's method
- Given u, the v coordinate is sampled by solving PV(v|u) = ξv, where PV(v|u) is the conditional CDF of v (a quadratic in v), and ξv is another uniform random number. This involves solving a quadratic equation and choosing the correct solution (v+ or v-) based on a specific condition related to a parameter τ(u, Φu, Φv)
- The sampled (u, v) coordinates are then converted to a 3D point location within the triangle

## Reference
[Efficient barycentric point sampling on meshes](https://arxiv.org/pdf/1708.07559)
