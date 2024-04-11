#include "kas_utils/depth_to_point_cloud.h"

#include <vector>
#include <utility>
#include <type_traits>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>


namespace py = pybind11;

namespace kas_utils {

template<typename T>
py::array_t<float> convertWrapper(const DepthToPointCloud<std::pair<float*, int>>& self,
    py::array_t<T, py::array::c_style> depth_array)
{
    static_assert(
        std::is_same<T, std::uint16_t>::value ||
        std::is_same<T, float>::value ||
        std::is_same<T, double>::value);

    const py::buffer_info buf_info = depth_array.request();
    std::vector<size_t> steps(buf_info.ndim - 1);
    for (int i = 0; i < buf_info.ndim - 1; i++)
    {
        steps[i] = buf_info.strides[i];
    }
    int type;
    if constexpr(std::is_same<T, std::uint16_t>::value)
    {
        type = CV_16UC1;
    }
    if constexpr(std::is_same<T, float>::value)
    {
        type = CV_32FC1;
    }
    if constexpr(std::is_same<T, double>::value)
    {
        type = CV_64FC1;
    }
    cv::Mat depth(
        std::vector<int>{buf_info.shape.begin(), buf_info.shape.end()},
        type, buf_info.ptr, steps.data());

    float* point_cloud;
    int points_number;
    std::tie(point_cloud, points_number) = self.convert(depth);

    py::capsule point_cloud_handler(point_cloud,
        [](void* ptr) {
            float* point_cloud = reinterpret_cast<float*>(ptr);
            delete[] point_cloud;
        });
    py::array_t<float> point_cloud_array(
        std::vector<ssize_t>{points_number, 3},
        std::vector<ssize_t>{sizeof(float) * 3, sizeof(float)},
        point_cloud, point_cloud_handler);

    return point_cloud_array;
}


void setCameraIntrinsicsWrapper(DepthToPointCloud<std::pair<float*, int>>& self,
    float fx, float fy, float cx, float cy)
{
    self.setCameraIntrinsics(fx, fy, cx, cy);
}


void setPoolSizeWrapper(DepthToPointCloud<std::pair<float*, int>>& self, int pool_size)
{
    self.setPoolSize(pool_size);
}


PYBIND11_MODULE(py_depth_to_point_cloud, m) {
    py::class_<DepthToPointCloud<std::pair<float*, int>>>(m, "DepthToPointCloud")
        .def(py::init<float, float, float, float, int>())
        .def("convert", &convertWrapper<std::uint16_t>)
        .def("convert", &convertWrapper<float>)
        .def("convert", &convertWrapper<double>)
        .def("set_camera_intrinsics", &setCameraIntrinsicsWrapper)
        .def("set_pool_size", &setPoolSizeWrapper);
}

}