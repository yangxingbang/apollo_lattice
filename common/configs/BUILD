load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "truck_config_helper",
    srcs = ["truck_config_helper.cc"],
    hdrs = ["truck_config_helper.h"],
    deps = [
        "//modules/planning/common:planning_gflags",
        "//cyber/common:file",
        "//cyber/common:macros",
        "//modules/planning/proto:truck_config_proto",
        "//modules/common/util",
        "@glog",
    ],
)

cpplint()
