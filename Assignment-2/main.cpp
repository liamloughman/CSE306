#include <vector>
#include <iostream>
#include <cmath>
#include <random>

static std::default_random_engine engine(10);
static std::uniform_real_distribution<double> uniform(0.0, 1.0);

class Vector {
public:
	explicit Vector(double x = 0, double y = 0) {
		data[0] = x;
		data[1] = y;
	}
	double norm2() const {
		return data[0] * data[0] + data[1] * data[1];
	}
	double norm() const {
		return sqrt(norm2());
	}
	Vector normalize() const {
		double n = norm();
		return Vector(data[0] / n, data[1] / n);
	}
	double operator[](int i) const { return data[i]; };
	double& operator[](int i) { return data[i]; };
	double data[2];
};

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1]);
}
Vector operator*(const double a, const Vector& b) {
	return Vector(a*b[0], a*b[1]);
}
Vector operator*(const Vector& a, const double b) {
	return Vector(a[0]*b, a[1]*b);
}
Vector operator*(const Vector& a, const Vector& b) {
    return Vector(a[0]*b[0], a[1]*b[1]);
}
Vector operator/(const Vector& a, const double b) {
	return Vector(a[0] / b, a[1] / b);
}
double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1];
}

class Polygon {  
public:

	std::vector<Vector> vertices;

	Polygon() {};

	explicit Polygon(const std::vector<Vector> &vertices) {
        this->vertices = vertices;
    };

    void add(const Vector& a) {
        this->vertices.push_back(a);
    };
};	

void save_svg(const std::vector<Polygon> &polygons, std::string filename, std::string fillcol) {
    FILE* f = fopen(filename.c_str(), "w+");
    fprintf(f, "<svg xmlns = \"http://www.w3.org/2000/svg\" width = \"1000\" height = \"1000\">\n");
    for (uint i=0; i<polygons.size(); i++) {
        fprintf(f, "<g>\n");
        fprintf(f, "<polygon points = \"");
        for (uint j = 0; j < polygons[i].vertices.size(); j++) {
            fprintf(f, "%3.3f, %3.3f ", (polygons[i].vertices[j][0] * 1000), (1000 - polygons[i].vertices[j][1] * 1000));
        }
        fprintf(f, "\"\nfill = \"%s\" stroke = \"black\"/>\n", fillcol.c_str());
        fprintf(f, "</g>\n");
    }
    fprintf(f, "</svg>\n");
    fclose(f);
}

void save_svg_animated(const std::vector<Polygon> &polygons, std::string filename, int frameid, int nbframes) {
    FILE* f;
    if (frameid == 0) {
        f = fopen(filename.c_str(), "w+");
        fprintf(f, "<svg xmlns = \"http://www.w3.org/2000/svg\" width = \"1000\" height = \"1000\">\n");
        fprintf(f, "<g>\n");
    } else {
        f = fopen(filename.c_str(), "a+");
    }
    fprintf(f, "<g>\n");
    for (int i = 0; i < polygons.size(); i++) {
        fprintf(f, "<polygon points = \""); 
        for (int j = 0; j < polygons[i].vertices.size(); j++) {
            fprintf(f, "%3.3f, %3.3f ", (polygons[i].vertices[j][0] * 1000), (1000-polygons[i].vertices[j][1] * 1000));
        }
        fprintf(f, "\"\nfill = \"none\" stroke = \"black\"/>\n");
    }
    fprintf(f, "<animate\n");
    fprintf(f, "	id = \"frame%u\"\n", frameid);
    fprintf(f, "	attributeName = \"display\"\n");
    fprintf(f, "	values = \"");
    for (int j = 0; j < nbframes; j++) {
        if (frameid == j) {
            fprintf(f, "inline");
        } else {
            fprintf(f, "none");
        }
        fprintf(f, ";");
    }
    fprintf(f, "none\"\n	keyTimes = \"");
    for (int j = 0; j < nbframes; j++) {
        fprintf(f, "%2.3f", j / (double)(nbframes));
        fprintf(f, ";");
    }
    fprintf(f, "1\"\n	dur = \"5s\"\n");
    fprintf(f, "	begin = \"0s\"\n");
    fprintf(f, "	repeatCount = \"indefinite\"/>\n");
    fprintf(f, "</g>\n");
    if (frameid == nbframes - 1) {
        fprintf(f, "</g>\n");
        fprintf(f, "</svg>\n");
    }
    fclose(f);
}

Vector intersect(const Vector &A, const Vector &B, const std::vector<Vector> &edges) {
    Vector u = edges[0];
    Vector v = edges[1];
    Vector N = Vector(v[1] - u[1], u[0] - v[0]).normalize();
    double t = dot((u - A), N) / dot((B - A), N);
    if (0 <= t && t <= 1) {
        return A + (B - A)*t;
    }
    return Vector();
}

bool inside(const Vector &P, const std::vector<Vector> &edges) {
    Vector u = edges[0];
    Vector v = edges[1];
    Vector N = Vector(v[1] - u[1], u[0] - v[0]).normalize();
    if (dot((P - u), N) <= 0) {
        return true;
    }
    return false;
}

Polygon clipPolygon(Polygon &subject_polygon, const Polygon &clip_polygon) {
    Polygon out_polygon;
    for (size_t i = 0; i < clip_polygon.vertices.size(); i++) {
        Vector u = clip_polygon.vertices[i];
        Vector v;
        if (i > 0) {
            v = clip_polygon.vertices[i - 1];
        } else {
            v = clip_polygon.vertices[clip_polygon.vertices.size() - 1];
        }
        out_polygon = Polygon();
        for (size_t j = 0; j < subject_polygon.vertices.size(); j++) {
            Vector cur_vertex = subject_polygon.vertices[j];
            Vector prev_vertex = subject_polygon.vertices[(j > 0) ? (j - 1): (subject_polygon.vertices.size() - 1)];
            Vector intersection = intersect(prev_vertex, cur_vertex, {u, v});
            if (inside(cur_vertex, {u, v})) {
                if (!inside(prev_vertex, {u, v})) {
                    out_polygon.add(intersection);
                }
                out_polygon.add(cur_vertex);
            } else if (inside(prev_vertex, {u, v})) {
                out_polygon.add(intersection);
            }
        }
        subject_polygon = out_polygon;
    }
    return out_polygon;
}

Vector voronoi_intersect(const Vector &A, const Vector &B, const std::vector<Vector> &edges) {
    Vector u = edges[0];
    Vector v = edges[1];
    Vector N = (u + v)/2;
    double t = dot(N - A, u - v)/dot(B - A, u - v);
    if (t >= 0 && t <= 1) {
        return A + (B - A)*t;
    }
    return Vector();
}

bool voronoi_inside(const Vector &point, const std::vector<Vector> &edges) {
    Vector u = edges[0];
    Vector v = edges[1];
    Vector N = (u + v)/2;
    if (dot(point - N, v - u) < 0) {
        return true;
    }
    return false;
}

std::vector<Polygon> voronoi(const std::vector<Vector> &points, const Polygon &bounds) {
    std::vector<Polygon> out_polygons(points.size());
    #pragma omp parallel for
    for (size_t i = 0; i < points.size(); i++) {
        Polygon cur_edges = bounds;
        for (size_t j = 0; j < points.size(); j++) {
            if (i != j) {
                Polygon out_polygon;
                for (size_t k = 0; k < cur_edges.vertices.size(); k++) {
                    Vector cur_vertex = cur_edges.vertices[k];
                    Vector prev_vertex = cur_edges.vertices[(k > 0) ? (k - 1): (cur_edges.vertices.size() - 1)];
                    Vector intersection = voronoi_intersect(prev_vertex, cur_vertex, {points[i], points[j]});
                    if (voronoi_inside(cur_vertex, {points[i], points[j]})) {
                        if (!voronoi_inside(prev_vertex, {points[i], points[j]})) {
                            out_polygon.add(intersection);
                        }
                        out_polygon.add(cur_vertex);
                    }
                    else if (voronoi_inside(prev_vertex, {points[i], points[j]})) {
                        out_polygon.add(intersection);
                    }
                }
                cur_edges = out_polygon;
            }
        }
        out_polygons[i] = cur_edges;
    }
    return out_polygons;
}

int main() {

    // -- Polygon Clipping --

    // Polygon subject_polygon;
    // subject_polygon.add(Vector(0.85, 0.5));
    // subject_polygon.add(Vector(0.58, 0.56));
    // subject_polygon.add(Vector(0.61, 0.83));
    // subject_polygon.add(Vector(0.47, 0.6));
    // subject_polygon.add(Vector(0.22, 0.7));
    // subject_polygon.add(Vector(0.4, 0.5));
    // subject_polygon.add(Vector(0.22, 0.3));
    // subject_polygon.add(Vector(0.47, 0.4));
    // subject_polygon.add(Vector(0.61, 0.17));
    // subject_polygon.add(Vector(0.58, 0.44));

    // Polygon clip_polygon;
    // clip_polygon.add(Vector(0.3, 0.3));
    // clip_polygon.add(Vector(0.3, 0.7));
    // clip_polygon.add(Vector(0.7, 0.7)); 
    // clip_polygon.add(Vector(0.7, 0.3));

    // save_svg({subject_polygon, clip_polygon}, "initial.svg", "none");

    // Polygon result_polygon = clipPolygon(subject_polygon, clip_polygon);

    // save_svg({result_polygon}, "clipped.svg", "none");

    // -- Voronoi --

    int n = 10000;
    
    Polygon bounds;
    bounds.add(Vector(0., 0.));
    bounds.add(Vector(0., 1.));
    bounds.add(Vector(1., 1.));
    bounds.add(Vector(1., 0.));

    std::vector<double> weights(n, 1);
    std::vector<Vector> points(n);
    for (size_t i = 0; i < n; i++) {
        points[i] = Vector(uniform(engine), uniform(engine));
    }

    std::vector<Polygon> result_voronoi = voronoi(points, bounds);

    std::string filename = "voronoi_" + std::to_string(n) + ".svg";

    save_svg(result_voronoi, filename, "none");

    return 0;
}