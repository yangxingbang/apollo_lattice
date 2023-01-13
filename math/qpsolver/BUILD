load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qp_solver",
    srcs = ["qp_solver.cc"],
    hdrs = ["qp_solver.h"],
    deps = [
        "@ipopt",
        "//cyber/common:log",
    ],
)

cpplint()
