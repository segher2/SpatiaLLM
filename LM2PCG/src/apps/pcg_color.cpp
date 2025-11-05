#include "pcg/params.hpp"
#include "pcg/ply_io.hpp"
#include "pcg/color_gmm.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <filesystem>
#include <iostream>
#include <random>
#include <cmath>
#include <algorithm>

namespace fs = std::filesystem;

static fs::path find_config_path() {
    const fs::path candidates[] = {
        fs::path("data/configs/default.yaml"),
        fs::path("../data/configs/default.yaml"),
        fs::path("../../data/configs/default.yaml")
    };
    for (const auto& p : candidates) if (fs::exists(p)) return p; return {};
}

// sRGB [0..255] -> Lab using D65/2°
static inline float srgb_to_linear(float c) {
    c = std::clamp(c, 0.0f, 1.0f);
    if (c <= 0.04045f) return c / 12.92f;
    return std::pow((c + 0.055f) / 1.055f, 2.4f);
}

static Eigen::Vector3f rgb_to_lab(const Eigen::Vector3f& rgb255) {
    float r = srgb_to_linear(rgb255[0] / 255.0f);
    float g = srgb_to_linear(rgb255[1] / 255.0f);
    float b = srgb_to_linear(rgb255[2] / 255.0f);
    float X = 0.4124564f*r + 0.3575761f*g + 0.1804375f*b;
    float Y = 0.2126729f*r + 0.7151522f*g + 0.0721750f*b;
    float Z = 0.0193339f*r + 0.1191920f*g + 0.9503041f*b;
    const float Xn = 0.95047f, Yn = 1.00000f, Zn = 1.08883f;
    auto f = [](float t) {
        const float eps = 0.008856f; const float k = 7.787f;
        return t > eps ? std::cbrtf(t) : (k * t + 16.0f / 116.0f);
    };
    float fx = f(X / Xn), fy = f(Y / Yn), fz = f(Z / Zn);
    float L = 116.0f * fy - 16.0f;
    float a = 500.0f * (fx - fy);
    float bb = 200.0f * (fy - fz);
    return Eigen::Vector3f(L, a, bb);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: pcg_color <cluster_ply> [sample_n]" << std::endl;
        return 1;
    }
    const fs::path ply_path = argv[1];

    pcg::ParamsConfig cfg; std::string err;
    pcg::load_params_from_file(find_config_path().string(), cfg, &err);
    const bool as_json = cfg.json_output;

    int sample_n = (argc >= 3) ? std::stoi(argv[2]) : cfg.color_sample_n;

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool has_color=false; std::string ioerr;
    if (!pcg::load_ply_xyz_or_rgb(ply_path.string(), xyz, rgb, has_color, &ioerr) || !has_color || !rgb) {
        std::cerr << "Error: input PLY has no RGB or failed to read: " << ioerr << std::endl;
        return 2;
    }

    const int N = static_cast<int>(rgb->size());
    if (N == 0) { std::cerr << "Empty cloud" << std::endl; return 3; }
    if (sample_n <= 0) sample_n = std::min(300, N);
    sample_n = std::min(sample_n, N);

    // Random subset sampling (deterministic seed for reproducibility)
    std::mt19937 rng(123);
    std::uniform_int_distribution<int> uni(0, N-1);
    std::vector<Eigen::Vector3f> X; X.reserve(sample_n);
    for (int i = 0; i < sample_n; ++i) {
        int idx = (sample_n == N) ? i : uni(rng);
        const auto& p = rgb->points[idx];
        X.emplace_back(p.r, p.g, p.b);
    }

    pcg::ColorGMMOptions opts; 
    opts.bic_extra_per_component = 0.0f; // no penalty in force-K mode
    opts.force_K = 3;                    // force K=3
    auto res = pcg::fit_color_gmm_bic(X, opts);
    if (res.K <= 0) { std::cerr << "GMM failed" << std::endl; return 4; }

    // Step 1: filter by weight and per-channel stddev thresholds
    const double min_w = cfg.color_min_weight;
    const double max_std = cfg.color_max_stddev;
    struct Comp { int idx; float w; Eigen::Vector3f mean; Eigen::Vector3f var; };
    std::vector<Comp> comps;
    for (int k = 0; k < res.K; ++k) {
        float w = res.weights[k];
        Eigen::Vector3f v = res.variances[k];
        Eigen::Vector3f sd(v[0] <= 0 ? 0 : std::sqrt(v[0]),
                           v[1] <= 0 ? 0 : std::sqrt(v[1]),
                           v[2] <= 0 ? 0 : std::sqrt(v[2]));
        bool ok = (w >= static_cast<float>(min_w)) && (sd[0] <= max_std && sd[1] <= max_std && sd[2] <= max_std);
        if (ok) comps.push_back({k, w, res.means[k], res.variances[k]});
    }

    if (comps.empty()) {
        if (as_json) {
            std::cout << "{\n  \"file\": \"" << ply_path.string() << "\",\n  \"M\": 0\n}\n";
        } else {
            std::cout << "Final M=0" << std::endl;
        }
        return 0;
    }

    if (comps.size() == 1) {
        const auto& c = comps[0];
        if (as_json) {
            std::cout << "{\n  \"file\": \"" << ply_path.string() << "\",\n  \"M\": 1,\n  \"components\": [ { \"weight\": " << c.w
                      << ", \"mean\": [" << c.mean[0] << ", " << c.mean[1] << ", " << c.mean[2]
                      << "], \"var\": [" << c.var[0] << ", " << c.var[1] << ", " << c.var[2] << "] } ]\n}\n";
        } else {
            std::cout << "Final M=1\n"
                      << "component 0: weight=" << c.w
                      << ", mean=[" << c.mean[0] << ", " << c.mean[1] << ", " << c.mean[2] << "]"
                      << ", var=[" << c.var[0] << ", " << c.var[1] << ", " << c.var[2] << "]\n";
        }
        return 0;
    }

    // Step 2: merge by Lab ΔE*76 single breakpoint (< color_deltaE_keep merges)
    const double dE_thr  = cfg.color_deltaE_keep;
    const int M0 = static_cast<int>(comps.size());
    std::vector<bool> alive(M0, true);
    std::vector<Eigen::Vector3f> labs(M0);
    for (int i = 0; i < M0; ++i) labs[i] = rgb_to_lab(comps[i].mean);

    for (int i = 0; i < M0; ++i) if (alive[i]) {
        for (int j = i + 1; j < M0; ++j) if (alive[j]) {
            Eigen::Vector3f d = labs[i] - labs[j];
            double dE = std::sqrt(d.dot(d));
            if (dE < dE_thr) {
                if (comps[i].w >= comps[j].w) alive[j] = false; else alive[i] = false;
            }
        }
    }

    std::vector<Comp> final;
    for (int i = 0; i < M0; ++i) if (alive[i]) final.push_back(comps[i]);

    if (as_json) {
        std::cout << "{\n  \"file\": \"" << ply_path.string() << "\",\n  \"M\": " << final.size() << ",\n  \"components\": [\n";
        for (size_t t = 0; t < final.size(); ++t) {
            const auto& c = final[t];
            std::cout << "    { \"weight\": " << c.w
                      << ", \"mean\": [" << c.mean[0] << ", " << c.mean[1] << ", " << c.mean[2]
                      << "], \"var\": [" << c.var[0] << ", " << c.var[1] << ", " << c.var[2] << "] }";
            if (t + 1 < final.size()) std::cout << ",";
            std::cout << "\n";
        }
        std::cout << "  ]\n}\n";
    } else {
        std::cout << "Final M=" << final.size() << "\n";
        for (size_t t = 0; t < final.size(); ++t) {
            const auto& c = final[t];
            std::cout << "component " << t
                      << ": weight=" << c.w
                      << ", mean=[" << c.mean[0] << ", " << c.mean[1] << ", " << c.mean[2] << "]"
                      << ", var=[" << c.var[0] << ", " << c.var[1] << ", " << c.var[2] << "]\n";
        }
    }

    return 0;
}
