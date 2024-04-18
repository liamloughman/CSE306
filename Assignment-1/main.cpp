#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <limits>
#include <random>
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

class Sphere {
public:
    Vector C; 
    double R; 
    Vector albedo;
    bool mirror;
    double refractive_index;
    bool invert_normal;
    Sphere(const Vector& C, double R, const Vector& albedo, bool mirror, double refractive_index = 1.0, bool invert_normal = false) {
        this->C = C;
        this->R = R;
        this->albedo = albedo;
        this->mirror = mirror;
        this->refractive_index = refractive_index;
        this->invert_normal = invert_normal;
    }
    Intersection intersect(const Ray& ray) const {
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
    std::vector<Sphere*> spheres; 
    Vector light_position;
    double light_intensity;  
    Scene(const Vector& light_position, double light_intensity) {
        this->light_position = light_position;
        this->light_intensity = light_intensity;
    }
    void addSphere(Sphere* sphere) {
        spheres.push_back(sphere);
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
    Intersection intersect(const Ray& ray) const {
        Intersection closestIntersection;
        closestIntersection.intersected = false;
        double closestDistance = std::numeric_limits<double>::max();
        int sphereIndex = -1; 
        for (size_t i = 0; i < spheres.size(); ++i) {
            const Sphere* sphere = spheres[i];
            Intersection intersection = sphere->intersect(ray);
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
        if (ray_depth < 0) return Vector(0., 0., 0.);
        Intersection intersection = intersect(ray);
        if (!intersection.intersected) {
            return Vector(0., 0., 0.); 
        }
        const double epsilon = 1e-4;
        Sphere& sphere = *spheres[intersection.index];
        Vector P = ray.O + ray.u * intersection.t; 
        Vector N = (P - sphere.C).normalize();
        if (sphere.mirror) {
            Vector reflection_direction = ray.u - 2 * dot(ray.u, N) * N;
            Ray reflected_ray(P + epsilon * N, reflection_direction);
            return getColor(reflected_ray, ray_depth - 1);
        }
        else if (sphere.refractive_index != 1.) {
            if (sphere.invert_normal) {
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

int main() {
    int W = 512;
    int H = 512;
    Vector Q(0, 0, 55);
    double fov = M_PI / 3;
    std::vector<unsigned char> image(W * H * 3, 0);
    double aspect_ratio = static_cast<double>(W) / H;
    double scale = tan(fov / 2);
    Scene scene(Vector(-10, 20, 40), 1e10);

    // Sphere white_sphere(Vector(0,0,0), 10, Vector(1, 1, 1), false); // center white sphere
    // scene.addSphere(&white_sphere);

    Sphere left_sphere(Vector(-10,0,20), 9, Vector(1,1,1), false, 1.5); //left full refractive sphere
    scene.addSphere(&left_sphere);
    Sphere white_sphere(Vector(0,0,5), 9, Vector(1, 1, 1), false); // center white sphere
    scene.addSphere(&white_sphere);
    Sphere right_sphere(Vector(10, 0, -10), 9, Vector(1, 1, 1), true); // left mirror sphere
    scene.addSphere(&right_sphere);

    // Sphere right_sphere(Vector(20, 0, 0), 10, Vector(1, 1, 1), false, 1.5); // right hollow refractive sphere
    // scene.addSphere(&right_sphere);
    // Sphere right_inner_sphere(Vector(20, 0, 0), 9.5, Vector(1, 1, 1), false, 1.5, true); // right inner refractive sphere
    // scene.addSphere(&right_inner_sphere);
    // Sphere left_sphere(Vector(-20, 0, 0), 10, Vector(1, 1, 1), true); // left mirror sphere
    // scene.addSphere(&left_sphere);
    // Sphere center_sphere(Vector(0,0,0), 10, Vector(1,1,1), false, 1.5); // center full refractive sphere
    // scene.addSphere(&center_sphere);

    Sphere ceiling = Sphere(Vector(0, 1000, 0), 940, Vector(1, 0, 0), false); // ceiling
    scene.addSphere(&ceiling);
    Sphere floor = Sphere(Vector(0, -1000, 0), 990, Vector(0, 0, 1), false); // floor 
    scene.addSphere(&floor);
    Sphere front = Sphere(Vector(0, 0, -1000), 940, Vector(0, 1, 0), false); // front
    scene.addSphere(&front);
    Sphere back = Sphere(Vector(0, 0, 1000), 940, Vector(1, 0, 1), false); // back
    scene.addSphere(&back);
    Sphere left = Sphere(Vector(1000, 0, 0), 940, Vector(1, 1, 0), false); // left
    scene.addSphere(&left);
    Sphere right = Sphere(Vector(-1000, 0, 0), 940, Vector(0, 1, 1), false); // right
    scene.addSphere(&right);
    double gamma = 2.2;
    int ray_depth = 5;
    // int K = 1000;
    int K = 32;
    #pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            Vector color_tmp(0., 0., 0.);
            double x, y;
            for (int k = 0; k< K; ++k) {
                boxMuller(0.5, x, y);
                Vector pixel = Vector(Q[0]+(j+x)+0.5-W/2, Q[1]-(i+y)-0.5+H/2, Q[2]-W/(2*tan(fov/2)));
                Ray ray(Q, (pixel-Q).normalize());
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
    stbi_write_png("image.png", W, H, 3, &image[0], 0);
    return 0;
}