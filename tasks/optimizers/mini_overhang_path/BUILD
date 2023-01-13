load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mini_overhang_path_optimizer",
    srcs = ["mini_overhang_path_optimizer.cc"],
    hdrs = ["mini_overhang_path_optimizer.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/math:cartesian_frenet_conversion",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_boundary",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/lattice/trajectory_generation:trajectory1d_generator",
        "//modules/planning/math:polynomial_xd",
        "//modules/planning/math/curve1d:polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_path_problem",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/tasks/optimizers:path_optimizer",
        ":mini_overhang_path_truck_problem",
        ":mini_overhang_path_car_problem",
        "//modules/planning/common:tractor_trailer_model",
        "@eigen",
    ],
)

cc_library(
    name = "mini_overhang_path_truck_problem",
    srcs = ["mini_overhang_path_truck_problem.cc"],
    hdrs = ["mini_overhang_path_truck_problem.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "@eigen",
        "//modules/planning/reference_line",
        "//modules/planning/math/qpsolver:qp_solver",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/common/math:geometry",
        "//modules/planning/common/configs:truck_config_helper",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/common/path:discretized_path",
    ],
)

cc_library(
    name = "mini_overhang_path_car_problem",
    srcs = ["mini_overhang_path_car_problem.cc"],
    hdrs = ["mini_overhang_path_car_problem.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "@eigen",
        "//modules/planning/reference_line",
        "//modules/planning/math/qpsolver:qp_solver",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/common/math:geometry",
        "//modules/planning/common/configs:truck_config_helper",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/common/path:discretized_path",
    ],
)

cpplint()
