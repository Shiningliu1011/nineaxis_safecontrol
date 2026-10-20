#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <fcl/config.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/convex.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace {

constexpr double kTolerance = 1e-12;

class Body {
 public:
  explicit Body(std::shared_ptr<fcl::CollisionGeometryd> geometry)
      : object(std::move(geometry)) {}

  void set_transform(const Eigen::Matrix3d& rotation,
                     const Eigen::Vector3d& translation) {
    if (!rotation.allFinite() || !translation.allFinite() ||
        (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-9 ||
        std::abs(rotation.determinant() - 1.0) > 1e-9) {
      throw std::invalid_argument("FCL transform must be finite and orthonormal");
    }
    fcl::Transform3d transform = fcl::Transform3d::Identity();
    transform.linear() = rotation;
    transform.translation() = translation;
    object.setTransform(transform);
  }

  fcl::CollisionObjectd object;
};

std::shared_ptr<Body> make_box(const Eigen::Vector3d& size) {
  if (!size.allFinite() || (size.array() <= 0).any()) {
    throw std::invalid_argument("FCL box dimensions must be positive");
  }
  return std::make_shared<Body>(std::make_shared<fcl::Boxd>(size));
}

std::shared_ptr<Body> make_cylinder(double radius, double height) {
  if (!std::isfinite(radius) || !std::isfinite(height) || radius <= 0 || height <= 0) {
    throw std::invalid_argument("FCL cylinder dimensions must be positive");
  }
  return std::make_shared<Body>(std::make_shared<fcl::Cylinderd>(radius, height));
}

std::shared_ptr<Body> make_convex(
    const Eigen::Ref<const Eigen::MatrixXd>& vertices,
    const Eigen::Ref<const Eigen::MatrixXi>& triangles) {
  if (vertices.cols() != 3 || vertices.rows() < 4 || !vertices.allFinite() ||
      triangles.cols() != 3 || triangles.rows() < 4) {
    throw std::invalid_argument("FCL convex mesh requires finite vertices and triangles");
  }
  auto points = std::make_shared<std::vector<fcl::Vector3d>>();
  auto faces = std::make_shared<std::vector<int>>();
  points->reserve(vertices.rows());
  faces->reserve(triangles.rows() * 4);
  for (Eigen::Index i = 0; i < vertices.rows(); ++i) {
    points->push_back(vertices.row(i).transpose());
  }
  for (Eigen::Index i = 0; i < triangles.rows(); ++i) {
    faces->push_back(3);
    for (Eigen::Index j = 0; j < 3; ++j) {
      int index = triangles(i, j);
      if (index < 0 || index >= vertices.rows()) {
        throw std::invalid_argument("FCL convex mesh index is out of range");
      }
      faces->push_back(index);
    }
  }
  return std::make_shared<Body>(std::make_shared<fcl::Convexd>(
      points, static_cast<int>(triangles.rows()), faces, true));
}

Eigen::Matrix<double, 1, 8> measure(const Body& first, const Body& second) {
  Eigen::Matrix<double, 1, 8> values = Eigen::Matrix<double, 1, 8>::Zero();
  fcl::CollisionRequestd collision_request;
  collision_request.gjk_tolerance = kTolerance;
  fcl::CollisionResultd collision;
  fcl::collide(&first.object, &second.object, collision_request, collision);
  if (collision.isCollision()) {
    values[0] = 1;
    return values;
  }
  fcl::DistanceRequestd request(true, false, 0.0, 0.0, kTolerance, fcl::GST_LIBCCD);
  fcl::DistanceResultd result;
  double distance = fcl::distance(&first.object, &second.object, request, result);
  if (!std::isfinite(distance) || distance < 0) {
    throw std::runtime_error("FCL distance query returned an invalid distance");
  }
  if (distance <= kTolerance) {
    values[0] = 1;
    return values;
  }
  if (!result.nearest_points[0].allFinite() || !result.nearest_points[1].allFinite()) {
    throw std::runtime_error("FCL distance query returned nonfinite nearest points");
  }
  values[1] = distance;
  values.segment<3>(2) = result.nearest_points[0].transpose();
  values.segment<3>(5) = result.nearest_points[1].transpose();
  return values;
}

py::array_t<double> measure_poses(
    const std::vector<std::shared_ptr<Body>>& moving,
    const std::vector<std::shared_ptr<Body>>& fixed,
    const py::array_t<double, py::array::c_style | py::array::forcecast>& poses,
    const Eigen::Ref<const Eigen::MatrixXi>& pairs) {
  if (poses.ndim() != 4 || poses.shape(1) != static_cast<py::ssize_t>(moving.size()) ||
      poses.shape(2) != 4 || poses.shape(3) != 4 || pairs.cols() != 2) {
    throw std::invalid_argument("FCL batch poses must have shape (N, bodies, 4, 4)");
  }
  std::vector<std::shared_ptr<Body>> bodies = moving;
  bodies.insert(bodies.end(), fixed.begin(), fixed.end());
  if ((pairs.array() < 0).any() || (pairs.array() >= static_cast<int>(bodies.size())).any()) {
    throw std::invalid_argument("FCL batch pair index is out of range");
  }
  py::array_t<double> output(
      {poses.shape(0), static_cast<py::ssize_t>(pairs.rows()), py::ssize_t{8}});
  const auto input = poses.unchecked<4>();
  auto values = output.mutable_unchecked<3>();
  {
    py::gil_scoped_release release;
    for (py::ssize_t sample = 0; sample < poses.shape(0); ++sample) {
      for (size_t body = 0; body < moving.size(); ++body) {
        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;
        for (int row = 0; row < 3; ++row) {
          translation[row] = input(sample, body, row, 3);
          for (int column = 0; column < 3; ++column) {
            rotation(row, column) = input(sample, body, row, column);
          }
        }
        for (int column = 0; column < 4; ++column) {
          if (input(sample, body, 3, column) != (column == 3 ? 1.0 : 0.0)) {
            throw std::invalid_argument("FCL batch pose must be a homogeneous transform");
          }
        }
        moving[body]->set_transform(rotation, translation);
      }
      for (Eigen::Index pair = 0; pair < pairs.rows(); ++pair) {
        const auto result = measure(*bodies[pairs(pair, 0)], *bodies[pairs(pair, 1)]);
        for (int field = 0; field < 8; ++field) {
          values(sample, pair, field) = result[field];
        }
      }
    }
  }
  return output;
}

}  // namespace

PYBIND11_MODULE(robot_static_fcl, module) {
  module.attr("distance_tolerance_m") = kTolerance;
  module.attr("fcl_version") = FCL_VERSION;
  py::class_<Body, std::shared_ptr<Body>>(module, "Body")
      .def_static("box", &make_box)
      .def_static("cylinder", &make_cylinder)
      .def_static("convex", &make_convex)
      .def("set_transform", &Body::set_transform);
  module.def("measure", &measure, py::call_guard<py::gil_scoped_release>());
  module.def("measure_poses", &measure_poses);
}
