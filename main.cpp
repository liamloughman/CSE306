#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <limits>

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
    Vector P; // Point of intersection
    Vector N; // Normal at the intersection
    Vector albedo;
};

class Ray {
public:
    explicit Ray(Vector& O, Vector& u) {
        this->O = O;
        this->u = u.normalize();
    }
    Vector O;
    Vector u;
};

class Sphere {
private:
    Vector C; // center of the sphere
    double R; // radius of the sphere
    Vector albedo; // color of the sphere

public:
    Sphere(const Vector& center, double radius, const Vector& color): C(center), R(radius), albedo(color) {}

    Intersection intersect(const Ray& ray) const {
        Intersection intersection;
        intersection.intersected = false; // Initialize as no intersection
        Vector O_C = ray.O - C;
        double a = 1; // since ray.u should be a unit vector
        double b = 2 * dot(ray.u, O_C);
        double c = dot(O_C, O_C) - R * R;
        double discriminant = b * b - 4 * a * c;

        if (discriminant >= 0) {
            double sqrt_discriminant = sqrt(discriminant);
            double t1 = (-b - sqrt_discriminant) / (2 * a);
            double t2 = (-b + sqrt_discriminant) / (2 * a);
            if (t1 >= 0 || t2 >= 0) {
                intersection.intersected = true;
                intersection.t = (t1 >= 0) ? t1 : t2; // Choose the nearest positive t
                intersection.P = ray.O + ray.u * intersection.t;
                intersection.N = (intersection.P - C).normalize(); // Assuming a normalized method exists
                intersection.albedo = albedo;
            }
        }
        return intersection;
    }
};

class Scene {
private:
    std::vector<Sphere*> spheres; 

public:
    Scene() {}

    void addSphere(Sphere* sphere) {
        spheres.push_back(sphere);
    }

    Intersection intersect(const Ray& ray) const {
        Intersection closestIntersection;
        closestIntersection.intersected = false;
        double closestDistance = std::numeric_limits<double>::max();

        for (const Sphere* sphere : spheres) {
            Intersection intersection = sphere->intersect(ray);
            if (intersection.intersected && intersection.t < closestDistance) {
                closestDistance = intersection.t;
                closestIntersection = intersection;
            }
        }

        return closestIntersection;
    }
};

int main() {
    int W = 512; // screen width
    int H = 512; // screen height
    Vector Q(0, 0, 55); // camera at (0, 0, 55)
    double fov = M_PI / 3; // field of view 60 degrees

    std::vector<unsigned char> image(W * H * 3, 0);

    double aspect_ratio = static_cast<double>(W) / H;
    double scale = tan(fov / 2);

    Scene scene;
    Sphere sphere(Vector(0, 0, 0), 10, Vector(1, 1, 1));
    scene.addSphere(&sphere);
    Sphere ceiling = Sphere(Vector(0, 1000, 0), 940, Vector(1, 0, 0));
	scene.addSphere(&ceiling);
    Sphere floor = Sphere(Vector(0, -1000, 0), 990, Vector(0, 0, 1));
	scene.addSphere(&floor);
    Sphere front = Sphere(Vector(0, 0, -1000), 940, Vector(0, 1, 0));
	scene.addSphere(&front);
    Sphere back = Sphere(Vector(0, 0, 1000), 940, Vector(1, 0, 1));
	scene.addSphere(&back);
    Sphere left = Sphere(Vector(1000, 0, 0), 940, Vector(1, 1, 0));
	scene.addSphere(&left);
    Sphere right = Sphere(Vector(-1000, 0, 0), 940, Vector(0, 1, 1));
	scene.addSphere(&right);

    Vector light_position(-10, 20, 40); // Example light source position
    double light_intensity = 2e10; // Example light source intensity
    double gamma = 2.2;

    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            double x = (2 * (j + 0.5) / static_cast<double>(W) - 1) * scale * aspect_ratio;
            double y = (1 - 2 * (i + 0.5) / static_cast<double>(H)) * scale;
            Vector dir = Vector(x, y, -1).normalize(); // direction for the ray
            Ray ray(Q, dir);

            Intersection intersection = scene.intersect(ray);
            if (intersection.intersected) {
                Vector P = intersection.P;
                Vector N = intersection.N;
                Vector offsetP = P + N * 1e-4;  // Offset point for shadow ray origin
                Vector L = light_position - P;
                double d = L.norm();
                L = L.normalize();
                double dotLN = std::max(dot(L, N), 0.0);
                Ray shadow_ray(offsetP, L);  // Shadow ray from slightly above the surface to the light

                Intersection shadow_intersection = scene.intersect(shadow_ray);
                double shadow = 1.0;
                if (shadow_intersection.intersected && shadow_intersection.t < d) {
                    shadow = 0.0; // Light is blocked
                }

                double intensity = light_intensity / (4 * M_PI * d * d);
                Vector color = shadow * intensity * dotLN * intersection.albedo / M_PI;
                color[0] = pow(color[0], 1.0 / gamma);
                color[1] = pow(color[1], 1.0 / gamma);
                color[2] = pow(color[2], 1.0 / gamma);

                image[(i * W + j) * 3 + 0] = static_cast<unsigned char>(std::min(color[0], 255.0));
                image[(i * W + j) * 3 + 1] = static_cast<unsigned char>(std::min(color[1], 255.0));
                image[(i * W + j) * 3 + 2] = static_cast<unsigned char>(std::min(color[2], 255.0));
            } else {
                image[(i * W + j) * 3 + 0] = 0;
                image[(i * W + j) * 3 + 1] = 0;
                image[(i * W + j) * 3 + 2] = 0;
            }
        }
    }

    stbi_write_png("image.png", W, H, 3, &image[0], 0);
    return 0;
}