#pragma once

// FlowStep and CameraCalib are supplied by the runtime.

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <cmath>
#include <vector>

struct BfGyroFlowResult {
    bool valid = false;
    int points = 0;
    double flow_body_x = 0.0;
    double flow_body_y = 0.0;
};

inline BfGyroFlowResult estimateBfGyroFlow(
    const FlowStep& raw,
    const CameraCalib& calib,
    const cv::Matx33d& C1_R_C0,
    double dt)
{
    BfGyroFlowResult out;

    if (!raw.valid || !(dt > 0.0 && dt < 0.2))
        return out;

    const auto& prev = raw.metric_prev_points;
    const auto& curr = raw.metric_curr_points;

    if (prev.size() != curr.size() || prev.size() < 20)
        return out;

    std::vector<cv::Point2f> a, b;

    cv::undistortPoints(prev, a, calib.K, calib.D);
    cv::undistortPoints(curr, b, calib.K, calib.D);

    cv::Mat A(static_cast<int>(a.size()) * 2, 4, CV_64F);
    cv::Mat rhs(static_cast<int>(a.size()) * 2, 1, CV_64F);

    for (size_t i = 0; i < a.size(); ++i) {
        const cv::Vec3d p0(a[i].x, a[i].y, 1.0);
        const cv::Vec3d pr = C1_R_C0 * p0;

        if (!std::isfinite(pr[2]) || pr[2] <= 1e-9)
            return out;

        const double xr = pr[0] / pr[2];
        const double yr = pr[1] / pr[2];

        const double du = b[i].x - xr;
        const double dv = b[i].y - yr;

        const int r = static_cast<int>(i) * 2;

        A.at<double>(r, 0) = 1.0;
        A.at<double>(r, 1) = 0.0;
        A.at<double>(r, 2) = xr;
        A.at<double>(r, 3) = -yr;
        rhs.at<double>(r, 0) = du;

        A.at<double>(r + 1, 0) = 0.0;
        A.at<double>(r + 1, 1) = 1.0;
        A.at<double>(r + 1, 2) = yr;
        A.at<double>(r + 1, 3) = xr;
        rhs.at<double>(r + 1, 0) = dv;
    }

    cv::Mat sol;
    if (!cv::solve(A, rhs, sol, cv::DECOMP_SVD) ||
        sol.rows != 4)
        return out;

    const double du = sol.at<double>(0, 0);
    const double dv = sol.at<double>(1, 0);

    const cv::Vec3d flow_cam(dv / dt, -du / dt, 0.0);

    const cv::Matx33d FLU_TO_FRD(
        1, 0, 0,
        0,-1, 0,
        0, 0,-1
    );

    const cv::Matx33d FRD_R_C =
        FLU_TO_FRD * calib.B_R_C;

    const cv::Vec3d flow_body = FRD_R_C * flow_cam;

    if (!std::isfinite(flow_body[0]) ||
        !std::isfinite(flow_body[1]))
        return out;

    out.valid = true;
    out.points = static_cast<int>(a.size());
    out.flow_body_x = flow_body[0];
    out.flow_body_y = flow_body[1];

    return out;
}
