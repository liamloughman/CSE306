#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <limits>
#include <random>
#include <string>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include<list>
static std::default_random_engine engine(10);
static std::uniform_real_distribution<double> uniform(0.0, 1.0);

class Vector {
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}
	double norm2() const {
		return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
	}
	double norm() const {
		return sqrt(norm2());
	}
	Vector normalize() const {
		double n = norm();
		return Vector(data[0] / n, data[1] / n, data[2] / n);
	}
	double operator[](int i) const { return data[i]; };
	double& operator[](int i) { return data[i]; };
	double data[3];
};

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector& b) {
	return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
	return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator*(const Vector& a, const Vector& b) {
    return Vector(a[0]*b[0], a[1]*b[1], a[2]*b[2]);
}
Vector operator/(const Vector& a, const double b) {
	return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

struct Intersection {
    bool intersected;
    double t;
    Vector P; 
    Vector N; 
    Vector albedo;
    int index;
    double refractive_index;
};

class Ray {
public:
    explicit Ray(const Vector& O, const Vector& u) {
        this->O = O;
        this->u = u.normalize();
    }
    Vector O;
    Vector u;
};

class Geometry {
public:
    Vector C; 
    double R; 
    Vector albedo;
    bool mirror;
    double refractive_index;
    bool invert_normal;
    virtual Intersection intersect(const Ray& ray) const = 0;
};

class Sphere : public Geometry{
public:
    Sphere(const Vector& C, double R, const Vector& albedo, bool mirror, double refractive_index = 1.0, bool invert_normal = false) {
        this->C = C;
        this->R = R;
        this->albedo = albedo;
        this->mirror = mirror;
        this->refractive_index = refractive_index;
        this->invert_normal = invert_normal;
    }
    Intersection intersect(const Ray& ray) const override {
        Intersection intersection;
        intersection.intersected = false; 
        Vector O_C = ray.O - C;
        double a = 1; 
        double b = 2 * dot(ray.u, O_C);
        double c = dot(O_C, O_C) - R * R;
        double discriminant = b * b - 4 * a * c;
        if (discriminant >= 0) {
            double sqrt_discriminant = sqrt(discriminant);
            double t1 = (-b - sqrt_discriminant) / (2 * a);
            double t2 = (-b + sqrt_discriminant) / (2 * a);
            if (t1 >= 0 || t2 >= 0) {
                intersection.intersected = true;
                intersection.t = (t1 >= 0) ? t1 : t2; 
                intersection.P = ray.O + ray.u * intersection.t;
                intersection.N = (intersection.P - C).normalize(); 
                intersection.albedo = this->albedo;
                intersection.refractive_index = this->refractive_index;
            }
        }
        return intersection;
    }
};

class Scene {
public:
    std::vector<Geometry*> geometries;
    Vector light_position;
    double light_intensity;  
    Scene(const Vector& light_position, double light_intensity) {
        this->light_position = light_position;
        this->light_intensity = light_intensity;
    }
    void addGeometry(Geometry* geometry) {
        geometries.push_back(geometry);
    }
    Vector random_cos(const Vector &N) {
        double r1 = ((double) rand() / (RAND_MAX));
        double r2 = ((double) rand() / (RAND_MAX));
        double x = sqrt(1 - r2)*cos(2.*M_PI*r1);
        double y = sqrt(1 - r2)*sin(2.*M_PI*r1);
        double z = sqrt(2);
        double min = abs(N[0]);
        int min_i = 0;
        for (int i = 0; i < 3; i++) {
            if (abs(N[i]) < min) {
                min = abs(N[i]);
                min_i = i;
            }
        }
        Vector T1;
        if (min_i == 0) {
            T1 = Vector(0., N[2], -N[1]).normalize();
        }
        else if (min_i == 1) {
            T1 = Vector(N[2], 0., -N[0]).normalize();
        }
        else if (min_i == 2) {
            T1 = Vector(N[1], -N[0], 0.).normalize();
        }
        Vector T2 = cross(T1, N);
        return T1*x + T2*y + N*z;
    }
    Intersection intersect(const Ray& ray) {
        Intersection closestIntersection;
        closestIntersection.intersected = false;
        double closestDistance = std::numeric_limits<double>::max();
        int sphereIndex = -1; 
        for (size_t i = 0; i < geometries.size(); i++) {
            const Geometry* geometry = geometries[i];
            Intersection intersection = geometry->intersect(ray);
            if (intersection.intersected && intersection.t < closestDistance) {
                closestDistance = intersection.t;
                closestIntersection = intersection;
                sphereIndex = i; 
            }
        }
        if (closestIntersection.intersected) {
            closestIntersection.index = sphereIndex;
        }
        return closestIntersection;
    }
    Vector getColor(const Ray& ray, int ray_depth) {
        if (ray_depth < 0) {
            return Vector(0., 0., 0.);
        }
        Intersection intersection = intersect(ray);
        if (!intersection.intersected) {
            return Vector(0., 0., 0.); 
        }
        const double epsilon = 1e-4;
        Geometry& geometry = *geometries[intersection.index];
        Vector N = intersection.N;
        Vector P = intersection.P + N*epsilon;
        if (geometry.mirror) {
            Vector reflection_direction = ray.u - 2 * dot(ray.u, N) * N;
            Ray reflected_ray(P + epsilon * N, reflection_direction);
            return getColor(reflected_ray, ray_depth - 1);
        }
        else if (geometry.refractive_index != 1.) {
            if (geometry.invert_normal) {
                N = (-1.)*N;
            }
            double n1, n2;
            if (dot(ray.u, N) > 0) {
                N = (-1.)*N;
                n1 = intersection.refractive_index;
                n2 = 1.;
            }
            else {
                n1 = 1.;
                n2 = intersection.refractive_index;
            }
            double k0 = pow((n1 - n2), 2.)/pow((n1 + n2), 2.);
            P = intersection.P - N*epsilon;
            if (1. - pow((n1/n2), 2.) * (1 - pow(dot(ray.u, N), 2.)) > 0) {
                Vector w_t  = (n1/n2)*(ray.u - dot(ray.u, N)*N);
                Vector w_n = (-1.)*N*sqrt(1 - pow((n1/n2), 2.)*(1 - pow(dot(ray.u, N), 2.)));
                Vector w = w_t + w_n;
                double x = ((double) rand() / (RAND_MAX));
                if (x < k0 + (1 - k0)*pow(1 - abs(dot(N,w)), 5.)) {
                    Ray reflected_ray = Ray(P, ray.u - (2*dot(intersection.N, ray.u))*intersection.N);
                    return getColor(reflected_ray, ray_depth - 1);
                }
                else {
                    Ray refracted_ray = Ray(P, w);
                    return getColor(refracted_ray, ray_depth - 1);
                }
            }
            else {
                Ray internal_refrected_ray = Ray(P, ray.u - (2*dot(intersection.N, ray.u))*intersection.N);
                return getColor(internal_refrected_ray, ray_depth - 1);
            }
        }
        else {
            double d = (this->light_position - P).norm();
            Vector light_direction = (this->light_position - P).normalize();
            Vector p_shadow = P + epsilon * N;
            Ray ray_shadow(p_shadow, light_direction);
            Intersection shadow_intersection = intersect(ray_shadow);
            double inShadow = (shadow_intersection.intersected && shadow_intersection.t < d) ? 0.0 : 1.0;
            Vector pixel_color = (this->light_intensity / (4 * M_PI * d * d)) * intersection.albedo * inShadow * std::max(0.0, dot(N, light_direction));
            Ray random_ray(P + epsilon * N, random_cos(N));
            pixel_color = pixel_color + intersection.albedo * getColor(random_ray, ray_depth - 1);
            return pixel_color;
        }
    }
};

void boxMuller ( double stdev , double& x , double &y ) {
    double r1 = uniform(engine);
    double r2 = uniform(engine);
    x = sqrt(-2*log(r1))*cos(2*M_PI*r2)*stdev;
    y = sqrt(-2*log(r1))*sin(2*M_PI*r2)*stdev;
}

class BoundingBox {
public:
    Vector B_min;
    Vector B_max;

    explicit BoundingBox(Vector B_min = Vector(), Vector B_max = Vector()) {
        this->B_min = B_min;
        this->B_max = B_max;
    }
};

struct Node {
    Node* left_child;
    Node* right_child;
    int starting_triangle;
    int ending_triangle;
    BoundingBox bounding_box;
};

class TriangleIndices {
public:
    TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
    };
    int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
    int uvi, uvj, uvk;  // indices within the uv coordinates array
    int ni, nj, nk;  // indices within the normals array
    int group;       // face group
};
 
 
class TriangleMesh : public Geometry{
public:
    double scaling_factor;
    Vector translation;
    BoundingBox bounding_box;
    Node* root;
   ~TriangleMesh() {}
    TriangleMesh(double scaling_factor, Vector translation, Vector albedo, double refractive_index = 1.0, bool mirror = false) {
        this->scaling_factor = scaling_factor;
        this->translation = translation;
        this->albedo = albedo;
        this->refractive_index = refractive_index;
        this->mirror = mirror;
        this->root = new Node;
    }
    
    void readOBJ(const char* obj) {
 
        char matfile[255];
        char grp[255];
 
        FILE* f;
        f = fopen(obj, "r");
        int curGroup = -1;
        while (!feof(f)) {
            char line[255];
            if (!fgets(line, 255, f)) break;
 
            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());
 
            if (line[0] == 'u' && line[1] == 's') {
                sscanf(line, "usemtl %[^\n]\n", grp);
                curGroup++;
            }
 
            if (line[0] == 'v' && line[1] == ' ') {
                Vector vec;
 
                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
                    col[0] = std::min(1., std::max(0., col[0]));
                    col[1] = std::min(1., std::max(0., col[1]));
                    col[2] = std::min(1., std::max(0., col[2]));
 
                    vertices.push_back(vec);
                    vertexcolors.push_back(col);
 
                } else {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n') {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't') {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f') {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;
                t.group = curGroup;
 
                char* consumedline = line + 1;
                int offset;
 
                nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9) {
                    if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                    if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                    if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                    if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                    if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                    if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                    if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                    if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                    if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                    indices.push_back(t);
                } else {
                    nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6) {
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                        if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                        if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                        if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                        indices.push_back(t);
                    } else {
                        nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                        if (nn == 3) {
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            indices.push_back(t);
                        } else {
                            nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                            if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                            if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }
 
                consumedline = consumedline + offset;
 
                while (true) {
                    if (consumedline[0] == '\n') break;
                    if (consumedline[0] == '\0') break;
                    nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.group = curGroup;
                    if (nn == 3) {
                        if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                        if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                        if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                        if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                        if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                        if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                        if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                        if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                        if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    } else {
                        nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                        if (nn == 2) {
                            if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                            if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                            if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                            if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                            if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                            if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        } else {
                            nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                            if (nn == 2) {
                                if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                                if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                                if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;                             
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            } else {
                                nn = sscanf(consumedline, "%u%n", &i3, &offset);
                                if (nn == 1) {
                                    if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                    if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                    if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                } else {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }
 
            }
 
        }
        fclose(f);
        this->build_BVH(this->root, 0, indices.size());
    }

    BoundingBox compute_bounding_box(int starting_triangle, int ending_triangle) {
        double min_x = MAXFLOAT, min_y = MAXFLOAT, min_z = MAXFLOAT;
        double max_x = -MAXFLOAT, max_y = -MAXFLOAT, max_z = -MAXFLOAT;
        for (int i = starting_triangle; i < ending_triangle; i++) {
            std::vector<Vector> triangle_vertices = {this->vertices[this->indices[i].vtxi], this->vertices[this->indices[i].vtxj], this->vertices[this->indices[i].vtxk]};
            for (const Vector& vertex : triangle_vertices) {
                Vector transformed_vertex = scaling_factor * vertex + translation;
                min_x = std::min(min_x, transformed_vertex[0]);
                max_x = std::max(max_x, transformed_vertex[0]);
                min_y = std::min(min_y, transformed_vertex[1]);
                max_y = std::max(max_y, transformed_vertex[1]);
                min_z = std::min(min_z, transformed_vertex[2]);
                max_z = std::max(max_z, transformed_vertex[2]);
            }
        }
        return BoundingBox(Vector(min_x, min_y, min_z), Vector(max_x, max_y, max_z));
    }

    bool bounding_box_intersection(const Ray &ray, const BoundingBox bounding_box, double &t) const {
        double tx0, ty0, tz0;
        double tx1, ty1, tz1;
        double t_B_min, t_B_max;
        Vector N;
        N = Vector(1,0,0);
        t_B_min = dot(bounding_box.B_min - ray.O, N) / dot(ray.u, N);
        t_B_max = dot(bounding_box.B_max - ray.O, N) / dot(ray.u, N);
        tx0 = std::min(t_B_min, t_B_max);
        tx1 = std::max(t_B_min, t_B_max);
        N = Vector(0,1,0);
        t_B_min = dot(bounding_box.B_min - ray.O, N) / dot(ray.u, N);
        t_B_max = dot(bounding_box.B_max - ray.O, N) / dot(ray.u, N);
        ty0 = std::min(t_B_min, t_B_max);
        ty1 = std::max(t_B_min, t_B_max);
        N = Vector(0,0,1);
        t_B_min = dot(bounding_box.B_min - ray.O, N) / dot(ray.u, N);
        t_B_max = dot(bounding_box.B_max - ray.O, N) / dot(ray.u, N);
        tz0 = std::min(t_B_min, t_B_max);
        tz1 = std::max(t_B_min, t_B_max);
        double first_intersection_t = std::max({tx0, ty0, tz0});
        double last_intersection_t = std::min({tx1, ty1, tz1});
      if (last_intersection_t > first_intersection_t > 0) {
        t = first_intersection_t;
        return true;
      }
      return false;
    }

    void build_BVH(Node *node, int starting_triangle, int ending_triangle) {
        node->bounding_box = compute_bounding_box(starting_triangle, ending_triangle);
        node->starting_triangle = starting_triangle;
        node->ending_triangle = ending_triangle;
        Vector diagonal = node->bounding_box.B_max - node->bounding_box.B_min;
        Vector middle_diagonal = node->bounding_box.B_min + 0.5*diagonal;
        int longest_axis = 0;
        double max = - MAXFLOAT;
        for (int i = 0; i < 3; i++) {
            if (abs(diagonal[i]) > max) {
                max = abs(diagonal[i]);
                longest_axis = i;
            }
        }
        int pivot_index = starting_triangle;
        for (int i = starting_triangle; i < ending_triangle; i++) {
            Vector v1 = this->vertices[this->indices[i].vtxi]*scaling_factor + translation;
            Vector v2 = this->vertices[this->indices[i].vtxj]*scaling_factor + translation;
            Vector v3 = this->vertices[this->indices[i].vtxk]*scaling_factor + translation;
            Vector barycenter = (v1 + v2 + v3)/3.;
            if (barycenter[longest_axis] < middle_diagonal[longest_axis]) {
                std::swap(indices[i], indices[pivot_index]);
                pivot_index++;
            }
        }
        if (pivot_index <= starting_triangle || pivot_index >= ending_triangle - 1 || ending_triangle - starting_triangle < 5) {
            return;
        }
        node->left_child = new Node();
        node->right_child = new Node();
        this->build_BVH(node->left_child, starting_triangle, pivot_index);
        this->build_BVH(node->right_child, pivot_index, ending_triangle);
    }

    Intersection intersect(const Ray &ray) const {
        Intersection intersection;
        intersection.intersected = false;
        double t;
        double min_t = MAXFLOAT;
        if (!bounding_box_intersection(ray, this->root->bounding_box, t)) {
            return intersection;
        }
        std::list<Node*> nodes_to_visit;
        nodes_to_visit.push_front(this->root);
        while(!nodes_to_visit.empty()) {
            Node* current_node = nodes_to_visit.back();
            nodes_to_visit.pop_back();
            if (current_node->left_child) {
                if (bounding_box_intersection(ray, current_node->left_child->bounding_box, t)) {
                    if (t < min_t) {
                        nodes_to_visit.push_back(current_node->left_child);
                    }
                }
                if (bounding_box_intersection(ray, current_node->right_child->bounding_box, t)) {
                    if (t < min_t) {
                        nodes_to_visit.push_back(current_node->right_child);
                    }
                }
            }
            else {
                Vector A, B, C, e_1, e_2, N;
                for (int i = current_node->starting_triangle; i < current_node->ending_triangle; i++) {
                    A = vertices[this->indices[i].vtxi]*scaling_factor + translation;
                    B = vertices[this->indices[i].vtxj]*scaling_factor + translation;
                    C = vertices[this->indices[i].vtxk]*scaling_factor + translation;
                    e_1 = B - A;
                    e_2 = C - A;
                    N = cross(e_1, e_2);
                    double beta = dot(e_2, cross(A - ray.O, ray.u))/dot(ray.u, N);
                    double gamma = - dot(e_1, cross(A - ray.O, ray.u))/dot(ray.u, N);
                    double alpha = 1.0 - beta - gamma;
                    if (alpha > 0.0 && beta > 0.0 && gamma > 0.0) {
                        double t = dot(A - ray.O, N)/dot(ray.u, N);
                        if (t > 0 && min_t > t) {
                            min_t = t;
                            intersection.intersected = true;
                            intersection.t = t;
                            intersection.P = A + e_1*beta + e_2*gamma;
                            intersection.N = N;
                            intersection.albedo = this->albedo;
                        }
                    }
                }
            }
        }
        return intersection;
    }
 
    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> vertexcolors;
};

int main() {

    auto start = std::chrono::high_resolution_clock::now();

    int W = 512;
    int H = 512;
    Vector Q(0, 0, 55);
    double fov = M_PI / 3;
    std::vector<unsigned char> image(W * H * 3, 0);
    double aspect_ratio = static_cast<double>(W) / H;
    double scale = tan(fov / 2);
    Scene scene(Vector(-10, 20, 40), 1e10);

    // Sphere* white_sphere = new Sphere(Vector(0,0,0), 10, Vector(1, 1, 1), false); // center white sphere
    // scene.addGeometry(white_sphere);

    // Sphere* left_sphere = new Sphere(Vector(-10,0,20), 9, Vector(1,1,1), false, 1.5); //left full refractive sphere
    // scene.addGeometry(left_sphere);
    // Sphere* white_sphere = new Sphere(Vector(0,0,5), 9, Vector(1, 1, 1), false); // center white sphere
    // scene.addGeometry(white_sphere);
    // Sphere* right_sphere = new Sphere(Vector(10, 0, -10), 9, Vector(1, 1, 1), true); // left mirror sphere
    // scene.addGeometry(right_sphere);
    // Sphere* top_sphere = new Sphere(Vector(0, 19, 5), 9, Vector(1, 1, 1), false, 1.5); // right hollow refractive sphere
    // scene.addGeometry(top_sphere);
    // Sphere* top_inner_sphere = new Sphere(Vector(0, 19, 5), 8.5, Vector(1, 1, 1), false, 1.5, true); // right inner refractive sphere
    // scene.addGeometry(top_inner_sphere);

    // Sphere* right_sphere(Vector(20, 0, 0), 10, Vector(1, 1, 1), false, 1.5); // right hollow refractive sphere
    // scene.addGeometry(right_sphere);
    // Sphere* right_inner_sphere(Vector(20, 0, 0), 9.5, Vector(1, 1, 1), false, 1.5, true); // right inner refractive sphere
    // scene.addGeometry(right_inner_sphere);
    // Sphere* left_sphere(Vector(-20, 0, 0), 10, Vector(1, 1, 1), true); // left mirror sphere
    // scene.addGeometry(left_sphere);
    // Sphere* center_sphere(Vector(0,0,0), 10, Vector(1,1,1), false, 1.5); // center full refractive sphere
    // scene.addGeometry(center_sphere);

    TriangleMesh* cat = new TriangleMesh(0.6, Vector(0, -10, 10), Vector(1., 1., 1.));
    cat->readOBJ("cat_model/cat.obj");
    scene.addGeometry(cat);

    // Sphere* top_sphere = new Sphere(Vector(-20, 25, -10), 9, Vector(1, 1, 1), false, 1.5); // right hollow refractive sphere
    // scene.addGeometry(top_sphere);
    // Sphere* top_inner_sphere = new Sphere(Vector(-20, 25, -10), 8.5, Vector(1, 1, 1), false, 1.5, true); // right inner refractive sphere
    // scene.addGeometry(top_inner_sphere);
    // Sphere* white_sphere = new Sphere(Vector(20, 25,-10), 9, Vector(1, 1, 1), false); // center white sphere
    // scene.addGeometry(white_sphere);
    // Sphere* left_sphere = new Sphere(Vector(-20,0,-10), 9, Vector(1,1,1), false, 1.5); //left full refractive sphere
    // scene.addGeometry(left_sphere);
    // Sphere* right_sphere = new Sphere(Vector(20, 0, -10), 9, Vector(1, 1, 1), true); // left mirror sphere
    // scene.addGeometry(right_sphere);

    Sphere* ceiling = new Sphere(Vector(0, 1000, 0), 940, Vector(1, 0, 0), false); // ceiling
    scene.addGeometry(ceiling);
    Sphere* floor = new Sphere(Vector(0, -1000, 0), 990, Vector(0, 0, 1), false); // floor 
    scene.addGeometry(floor);
    Sphere* front = new Sphere(Vector(0, 0, -1000), 940, Vector(0, 1, 0), false); // front
    scene.addGeometry(front);
    Sphere* back = new Sphere(Vector(0, 0, 1000), 940, Vector(1, 0, 1), false); // back
    scene.addGeometry(back);
    Sphere* left = new Sphere(Vector(1000, 0, 0), 940, Vector(1, 1, 0), false); // left
    scene.addGeometry(left);
    Sphere* right = new Sphere(Vector(-1000, 0, 0), 940, Vector(0, 1, 1), false); // right
    scene.addGeometry(right);

    double aperture_radius = 1.5;
    double focus_distance = 50;
    double gamma = 2.2;
    int ray_depth = 5;
    int K = 10;
    #pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < H; i++) {
        std::cout<<i<<std::endl;
        for (int j = 0; j < W; j++) {
            Vector color_tmp(0., 0., 0.);
            double x, y;
            for (int k = 0; k< K; k++) {
                boxMuller(0.5, x, y);
                double theta = 2*M_PI*uniform(engine);
                double aperture_shape = sqrt(uniform(engine))*aperture_radius;
                Ray ray(Q+Vector(aperture_shape*cos(theta), aperture_shape*sin(theta), 0), (Q+focus_distance*Vector(0.5+j+x-W*0.5, y-i+H*0.5-0.5, -W/(2*tan(fov/2))).normalize()-Q-Vector(aperture_shape*cos(theta), aperture_shape*sin(theta),0)).normalize());
                color_tmp = color_tmp + scene.getColor(ray, ray_depth);
            }

            Vector color = color_tmp/K;

            color[0] = pow(color[0], 1.0 / gamma);
            color[1] = pow(color[1], 1.0 / gamma);
            color[2] = pow(color[2], 1.0 / gamma);

            image[(i * W + j) * 3 + 0] = static_cast<unsigned char>(std::min(color[0], 255.0));
            image[(i * W + j) * 3 + 1] = static_cast<unsigned char>(std::min(color[1], 255.0));
            image[(i * W + j) * 3 + 2] = static_cast<unsigned char>(std::min(color[2], 255.0));
        }
    }
    // loop without depth of field
    // #pragma omp parallel for schedule(dynamic, 1)
    // for (int i = 0; i < H; ++i) {
    //     for (int j = 0; j < W; ++j) {
    //         Vector color_tmp(0., 0., 0.);
    //         double x, y;
    //         for (int k = 0; k< K; ++k) {
    //             boxMuller(0.5, x, y);
    //             Vector pixel = Vector(Q[0]+(j+x)+0.5-W/2, Q[1]-(i+y)-0.5+H/2, Q[2]-W/(2*tan(fov/2)));
    //             Ray ray(Q, (pixel-Q).normalize());
    //             color_tmp = color_tmp + scene.getColor(ray, ray_depth);
    //         }

    //         Vector color = color_tmp/K;

    //         color[0] = pow(color[0], 1.0 / gamma);
    //         color[1] = pow(color[1], 1.0 / gamma);
    //         color[2] = pow(color[2], 1.0 / gamma);

    //         image[(i * W + j) * 3 + 0] = static_cast<unsigned char>(std::min(color[0], 255.0));
    //         image[(i * W + j) * 3 + 1] = static_cast<unsigned char>(std::min(color[1], 255.0));
    //         image[(i * W + j) * 3 + 2] = static_cast<unsigned char>(std::min(color[2], 255.0));
    //     }
    // }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
    if (duration.count() < 60000) {
        std::cout<<"Time for rendering: "<<duration.count()/1000.<<" seconds."<<std::endl;
    }
    else {  
        std::cout<<"Time for rendering: "<<duration.count()/60000<<" minute(s) and "<< fmod(duration.count()/60000., 1.0)*60<<" seconds."<<std::endl;
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);
    return 0;
}