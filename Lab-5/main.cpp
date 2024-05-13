#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <algorithm>
#include <random>
#include <vector>

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

Vector random_direction() {
    double r1 = ((double) rand() / (RAND_MAX));
    double r2 = ((double) rand() / (RAND_MAX));
    double x = cos(2 * M_PI * r1) * sqrt(r2 * (1 - r2));
    double y = sin(2 * M_PI * r1) * sqrt(r2 * (1 - r2));
    double z = 1 - 2 * r2;
    return Vector(x, y, z);
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        printf("Invalid input. Please enter: input_image target_image iterations.\n");
        exit(1);
    }
    const char *input_image_name = argv[1];
    const char *target_name = argv[2];
    size_t iterations = atoi(argv[3]);
    int W, H, C, target_W, target_H, target_C;
    unsigned char *input_image = stbi_load(input_image_name, &W, &H, &C, 0);
    unsigned char *target_image = stbi_load(target_name, &target_W, &target_H, &target_C, 0);
    if (input_image == NULL) {
        printf("Invalid input image.\n");
        exit(1);
    }
    if (target_image == NULL) {
        printf("Invalid input image.\n");
        exit(1);
    }
    if (W != target_W || H != target_H || C != target_C) {
        printf("Invalid dimension and channels.\n");
        exit(1);
    }
    std::vector<std::pair<int, int>> projI(W * H);
    std::vector<std::pair<int, int>> projM(W * H);
    Vector pixel, target_pixel, v;
    for (size_t iter = 0; iter < iterations; iter++) {
        v = random_direction();

        for (size_t i = 0; i < W * H; i++) {
            unsigned char *I = input_image + C * i;
            unsigned char *M = target_image + target_C * i;
            pixel = Vector(*I, *(I + 1), *(I + 2));
            target_pixel = Vector(*M, *(M + 1), *(M + 2));
            projI[i] = std::pair<int, int>(dot(pixel, v), i);
            projM[i] = std::pair<int, int>(dot(target_pixel, v), i);
        }
        std::sort(projI.begin(), projI.end());
        std::sort(projM.begin(), projM.end());
        for (size_t i = 0; i < W * H; i++) {
            unsigned char *pixel_output = input_image + projI[i].second * C;
            pixel = Vector(*pixel_output, *(pixel_output + 1), *(pixel_output + 2)) + (projM[i].first - projI[i].first) * v;
            *pixel_output = pixel[0];
            *(pixel_output + 1) = pixel[1];
            *(pixel_output + 2) = pixel[2];
        }
    }
    stbi_write_png("output.png", W, H, C, &input_image[0], 0);
    return 0;
}