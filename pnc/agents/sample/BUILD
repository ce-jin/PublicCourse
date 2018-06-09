package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sample_agent",
    hdrs = ["sample_agent.h"],
    deps = [
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "@glog",
        "//homework5/map:map_lib",
        "//homework4:camera_lidar_fusion_utils",
        "@eigen//:eigen",
        "//common/proto:cc_object_labeling_3d_proto",
        "//common/utils/display:painter_widget_base",
        "//common/utils/file:path",
        "//common/utils/strings:format",
        "@qt//:qt_widgets",
		"//common/proto:cc_route_proto",
    ],
)
