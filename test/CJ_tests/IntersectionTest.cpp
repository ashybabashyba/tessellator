#include "gtest/gtest.h"
#include "MeshFixtures.h"

#include "filler/Filler.h"
#include "Slicer.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Intersection_traits.h>
#include <CGAL/enum.h>
#include <CGAL/intersections.h>
#include <vector>
#include <iostream>

using namespace meshlib;
using namespace tessellator;
using namespace filler;
using namespace meshFixtures;

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Plane_3 Plane_3;
typedef Kernel::Segment_3 Segment_3;
typedef Kernel::Triangle_3 Triangle_3;
typedef boost::variant<Segment_3, Point_3, std::vector<Point_3>, std::vector<Segment_3>> IntersectResult;


class IntersectionTest : public ::testing::Test {
public:
    static const std::size_t X{ 0 };
    static const std::size_t Y{ 1 };
    static const std::size_t Z{ 2 };

    static Mesh buildCubeMesh()
    {
        ///    5 --- 6
        ///  / |   / |
        /// 4 --- 7  |
        /// |  |  |  |
        /// |  1 -|- 2
        /// | /   | /
        /// 0 --- 3  
        Grid grid;
        grid[0] = { 0.0, 1.0 };
        grid[1] = { 0.0, 1.0 };
        grid[2] = { 0.0, 1.0, 2.0 };
        std::vector<Coordinate> coords(8);

        coords[0] = Coordinate{ {0.00, 0.00, 0.00} };
        coords[1] = Coordinate{ {1.00, 0.00, 0.00} };
        coords[2] = Coordinate{ {0.00, 1.00, 0.00} };
        coords[3] = Coordinate{ {1.00, 1.00, 0.00} };

        coords[4] = Coordinate{ {0.00, 0.00, 1.00} };
        coords[5] = Coordinate{ {1.00, 0.00, 1.00} };
        coords[6] = Coordinate{ {0.00, 1.00, 1.00} };
        coords[7] = Coordinate{ {1.00, 1.00, 1.00} };

        Element sqrface;
        sqrface.type = Element::Type::Surface;
        std::vector<Group> groups = { Group({std::vector<Element>(6, sqrface)}) };
        groups[0].elements[0].vertices = { 0, 3, 2, 1 };
        groups[0].elements[1].vertices = { 4, 5, 6, 7 };
        groups[0].elements[2].vertices = { 7, 6, 2, 3 };
        groups[0].elements[3].vertices = { 4, 7, 3, 0 };
        groups[0].elements[4].vertices = { 5, 4, 0, 1 };
        groups[0].elements[5].vertices = { 6, 5, 1, 2 };


        return Mesh{ grid, coords, groups };
    }

    static std::vector<Triangle_3> defineCubeTriangles() {
        Mesh cube = buildCubeMesh();
        return {
            Triangle_3(Point_3(cube.coordinates[0][0], cube.coordinates[0][1], cube.coordinates[0][2]),
                       Point_3(cube.coordinates[1][0], cube.coordinates[1][1], cube.coordinates[1][2]),
                       Point_3(cube.coordinates[2][0], cube.coordinates[2][1], cube.coordinates[2][2])),

            Triangle_3(Point_3(cube.coordinates[0][0], cube.coordinates[0][1], cube.coordinates[0][2]),
                       Point_3(cube.coordinates[3][0], cube.coordinates[3][1], cube.coordinates[3][2]),
                       Point_3(cube.coordinates[2][0], cube.coordinates[2][1], cube.coordinates[2][2])),

            Triangle_3(Point_3(cube.coordinates[0][0], cube.coordinates[0][1], cube.coordinates[0][2]),
                       Point_3(cube.coordinates[1][0], cube.coordinates[1][1], cube.coordinates[1][2]),
                       Point_3(cube.coordinates[4][0], cube.coordinates[4][1], cube.coordinates[4][2])),

            Triangle_3(Point_3(cube.coordinates[4][0], cube.coordinates[4][1], cube.coordinates[4][2]),
                       Point_3(cube.coordinates[1][0], cube.coordinates[1][1], cube.coordinates[1][2]),
                       Point_3(cube.coordinates[5][0], cube.coordinates[5][1], cube.coordinates[5][2])),

            Triangle_3(Point_3(cube.coordinates[0][0], cube.coordinates[0][1], cube.coordinates[0][2]),
                       Point_3(cube.coordinates[3][0], cube.coordinates[3][1], cube.coordinates[3][2]),
                       Point_3(cube.coordinates[4][0], cube.coordinates[4][1], cube.coordinates[4][2])),

            Triangle_3(Point_3(cube.coordinates[4][0], cube.coordinates[4][1], cube.coordinates[4][2]),
                       Point_3(cube.coordinates[3][0], cube.coordinates[3][1], cube.coordinates[3][2]),
                       Point_3(cube.coordinates[7][0], cube.coordinates[7][1], cube.coordinates[7][2])),

            Triangle_3(Point_3(cube.coordinates[6][0], cube.coordinates[6][1], cube.coordinates[6][2]),
                       Point_3(cube.coordinates[3][0], cube.coordinates[3][1], cube.coordinates[3][2]),
                       Point_3(cube.coordinates[2][0], cube.coordinates[2][1], cube.coordinates[2][2])),

            Triangle_3(Point_3(cube.coordinates[7][0], cube.coordinates[7][1], cube.coordinates[7][2]),
                       Point_3(cube.coordinates[3][0], cube.coordinates[3][1], cube.coordinates[3][2]),
                       Point_3(cube.coordinates[6][0], cube.coordinates[6][1], cube.coordinates[6][2])),

            Triangle_3(Point_3(cube.coordinates[5][0], cube.coordinates[5][1], cube.coordinates[5][2]),
                       Point_3(cube.coordinates[1][0], cube.coordinates[1][1], cube.coordinates[1][2]),
                       Point_3(cube.coordinates[2][0], cube.coordinates[2][1], cube.coordinates[2][2])),

            Triangle_3(Point_3(cube.coordinates[5][0], cube.coordinates[5][1], cube.coordinates[5][2]),
                       Point_3(cube.coordinates[6][0], cube.coordinates[6][1], cube.coordinates[6][2]),
                       Point_3(cube.coordinates[2][0], cube.coordinates[2][1], cube.coordinates[2][2])),

            Triangle_3(Point_3(cube.coordinates[5][0], cube.coordinates[5][1], cube.coordinates[5][2]),
                       Point_3(cube.coordinates[6][0], cube.coordinates[6][1], cube.coordinates[6][2]),
                       Point_3(cube.coordinates[4][0], cube.coordinates[4][1], cube.coordinates[4][2])),

            Triangle_3(Point_3(cube.coordinates[5][0], cube.coordinates[5][1], cube.coordinates[5][2]),
                       Point_3(cube.coordinates[6][0], cube.coordinates[6][1], cube.coordinates[6][2]),
                       Point_3(cube.coordinates[7][0], cube.coordinates[7][1], cube.coordinates[7][2])),

                    };
    }

    static Segment_3 unite_collinear_segments(const Segment_3& s1, const Segment_3& s2) {
        if (!CGAL::collinear(s1.source(), s1.target(), s2.source()) ||
            !CGAL::collinear(s1.source(), s1.target(), s2.target())) {
            throw std::invalid_argument("The segments are not collinear.");
        }

        Point_3 min_point = std::min({ s1.source(), s1.target(), s2.source(), s2.target() });
        Point_3 max_point = std::max({ s1.source(), s1.target(), s2.source(), s2.target() });

        return Segment_3(min_point, max_point);
    }

    static boost::optional<IntersectResult> findMaxIntersection(const Segment_3& segment, const std::vector<Triangle_3>& triangles) {
        std::set<Point_3> intersection_points;
        std::vector<Segment_3> intersection_segments; 
        boost::optional<IntersectResult> max_intersection;

        for (const auto& triangle : triangles) {
            auto result = CGAL::intersection(segment, triangle);
            if (result) {
                if (const Segment_3* l = boost::get<Segment_3>(&*result)) {
                    Segment_3 segment_to_store = *l;
                    intersection_segments.push_back(segment_to_store); 
                }
                else if (const Point_3* p = boost::get<Point_3>(&*result)) {
                    intersection_points.insert(*p);
                }
                else if (!max_intersection) {
                    max_intersection = result;
                }
            }
        }
        if (!intersection_points.empty() && intersection_segments.empty()) {
            std::vector<Point_3> unique_points(intersection_points.begin(), intersection_points.end());
            return unique_points; 
        }
        if (!intersection_segments.empty()) {
            auto it = intersection_segments.begin(); 
            Segment_3 s1 = *it; 

            ++it; 
            Segment_3 s2 = *it; 

            Segment_3 united_segment = unite_collinear_segments(s1, s2); 
            return united_segment; 
        }
        return max_intersection;
    }
};

TEST_F(IntersectionTest, CubeFaceIntersections) {
    Mesh cube = buildCubeMesh();
    Segment_3 segment(Point_3(0.5, 0, 0), Point_3(0.5, 0, 4));

    std::vector<Triangle_3> triangles = defineCubeTriangles();

    auto max_intersection = findMaxIntersection(segment, triangles);

    if (max_intersection) {
        if (const Segment_3* l = boost::get<Segment_3>(&*max_intersection)) {
            std::cout << "Intersection is a Segment: " << *l << std::endl;
        }
        else if (const std::vector<Point_3>* vec = boost::get<std::vector<Point_3>>(&*max_intersection)) {
            for (const auto& pt : *vec) {
                std::cout << "Intersection in the point: " << pt << std::endl;
            }
        }
    }
    else {
        std::cout << "No intersection found." << std::endl;
    }
}