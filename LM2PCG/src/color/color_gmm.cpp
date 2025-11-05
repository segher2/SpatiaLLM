#include "pcg/color_gmm.hpp"

#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>

namespace pcg {

namespace {

static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

// Compute weighted mean and diag variance
static void weighted_stats(const std::vector<Eigen::Vector3f>& X, const std::vector<float>& w,
                           const std::vector<int>& idx, Eigen::Vector3f& mean, Eigen::Vector3f& var, float& wsum) {
    mean.setZero(); var.setZero(); wsum = 0.f;
    for (int i : idx) {
        float wi = w[i];
        mean += wi * X[i];
        wsum += wi;
    }
    if (wsum <= 0.f) { mean.setZero(); var.setOnes(); return; }
    mean /= wsum;
    for (int i : idx) {
        Eigen::Vector3f d = X[i] - mean;
        var += w[i] * d.cwiseProduct(d);
    }
    if (wsum > 0.f) {
        var /= wsum;
    } else {
        var.setOnes();
    }
}

// Simple kmeans++ init returning K centers
static std::vector<Eigen::Vector3f> kmeanspp(const std::vector<Eigen::Vector3f>& X, const std::vector<float>& w, int K, std::mt19937& rng) {
    std::uniform_int_distribution<int> uni(0, static_cast<int>(X.size()) - 1);
    std::vector<Eigen::Vector3f> centers; centers.reserve(K);
    centers.push_back(X[uni(rng)]);
    std::vector<float> d2(X.size(), std::numeric_limits<float>::max());
    for (int c = 1; c < K; ++c) {
        for (size_t i = 0; i < X.size(); ++i) {
            float dist2 = (X[i] - centers.back()).squaredNorm();
            d2[i] = std::min(d2[i], dist2);
        }
        float sum = 0.f; for (size_t i = 0; i < X.size(); ++i) sum += d2[i] * (w.empty() ? 1.f : w[i]);
        std::uniform_real_distribution<float> pick(0.f, sum);
        float r = pick(rng);
        size_t idx = 0; float acc = 0.f;
        for (; idx < X.size(); ++idx) { acc += d2[idx] * (w.empty() ? 1.f : w[idx]); if (acc >= r) break; }
        if (idx >= X.size()) idx = X.size() - 1;
        centers.push_back(X[idx]);
    }
    return centers;
}

static float logsumexp3(float a, float b, float c) {
    float m = std::max(a, std::max(b, c));
    return m + std::log(std::exp(a - m) + std::exp(b - m) + std::exp(c - m));
}

static float logsumexpN(const std::vector<float>& v) {
    float m = -std::numeric_limits<float>::infinity();
    for (float x : v) m = std::max(m, x);
    float s = 0.f; for (float x : v) s += std::exp(x - m);
    return m + std::log(s);
}

} // namespace

GMMResult fit_color_gmm_bic(const std::vector<Eigen::Vector3f>& X,
                            const std::vector<int>& w_in) {
    ColorGMMOptions opts; // default
    return fit_color_gmm_bic(X, opts, w_in);
}

GMMResult fit_color_gmm_bic(const std::vector<Eigen::Vector3f>& X,
                            const ColorGMMOptions& opts,
                            const std::vector<int>& w_in) {
    GMMResult best; best.K = 0;
    if (X.empty()) return best;
    const int N = static_cast<int>(X.size());
    std::vector<float> w(N, 1.f);
    if (!w_in.empty()) for (int i = 0; i < N; ++i) w[i] = std::max(1, w_in[i]);

    std::mt19937 rng(42);
    const float var_floor = 25.0f; // about (5 levels)^2 in 0..255 space
    const float dir_prior = 1e-3f; // small symmetric Dirichlet prior

    auto em_for_K = [&](int K) -> std::pair<float, GMMResult> {
        // init
        std::vector<Eigen::Vector3f> means = kmeanspp(X, w, K, rng);
        std::vector<Eigen::Vector3f> vars(K, Eigen::Vector3f::Constant(1000.f));
        std::vector<float> pis(K, 1.0f / K);

        // responsibilities [N x K]
        std::vector<float> gamma(N * K, 0.f);

        auto log_gauss = [&](const Eigen::Vector3f& x, const Eigen::Vector3f& m, const Eigen::Vector3f& v) {
            Eigen::Vector3f vv = v.cwiseMax(var_floor);
            Eigen::Vector3f d = x - m;
            float log_det = std::log(vv[0]) + std::log(vv[1]) + std::log(vv[2]);
            float quad = d[0]*d[0]/vv[0] + d[1]*d[1]/vv[1] + d[2]*d[2]/vv[2];
            return -0.5f * (log_det + quad + 3.0f * std::log(2.0f * static_cast<float>(M_PI)));
        };

        const int max_iter = 50;
        for (int it = 0; it < max_iter; ++it) {
            // E-step
            float ll = 0.f;
            for (int i = 0; i < N; ++i) {
                std::vector<float> logs(K);
                for (int k = 0; k < K; ++k) logs[k] = std::log(std::max(1e-8f, pis[k])) + log_gauss(X[i], means[k], vars[k]);
                float lse = (K==1) ? logs[0] : (K==2 ? logsumexpN(logs) : logsumexpN(logs));
                ll += w[i] * lse;
                for (int k = 0; k < K; ++k) gamma[i*K + k] = std::exp(logs[k] - lse);
            }
            // M-step with weights + Dirichlet prior
            std::vector<float> Nk(K, 0.f);
            for (int k = 0; k < K; ++k) {
                float denom = 0.f; Eigen::Vector3f num_m = Eigen::Vector3f::Zero();
                for (int i = 0; i < N; ++i) {
                    float g = gamma[i*K + k] * w[i];
                    denom += g;
                    num_m += g * X[i];
                }
                Nk[k] = denom;
                if (denom <= 0.f) {
                    std::uniform_int_distribution<int> uni(0, N-1);
                    means[k] = X[uni(rng)];
                    vars[k] = Eigen::Vector3f::Constant(1000.f);
                    continue;
                }
                means[k] = num_m / std::max(1e-6f, denom);
                Eigen::Vector3f num_v = Eigen::Vector3f::Zero();
                for (int i = 0; i < N; ++i) {
                    float g = gamma[i*K + k] * w[i];
                    Eigen::Vector3f d = X[i] - means[k];
                    num_v += g * d.cwiseProduct(d);
                }
                vars[k] = (num_v / std::max(1e-6f, denom)).cwiseMax(var_floor);
            }
            float Neff = 0.f; for (float wi : w) Neff += wi;
            for (int k = 0; k < K; ++k) {
                pis[k] = (Nk[k] + dir_prior) / std::max(1e-6f, Neff + K * dir_prior);
            }
        }

        // Final log-likelihood
        float ll = 0.f; float Neff = 0.f; for (float wi : w) Neff += wi;
        for (int i = 0; i < N; ++i) {
            std::vector<float> logs(K);
            for (int k = 0; k < K; ++k) logs[k] = std::log(std::max(1e-8f, pis[k])) +
                                                (-0.5f * (std::log(std::max(var_floor, vars[k][0])) +
                                                          std::log(std::max(var_floor, vars[k][1])) +
                                                          std::log(std::max(var_floor, vars[k][2])) +
                                                          (X[i]-means[k]).cwiseQuotient(vars[k].cwiseMax(var_floor)).dot(X[i]-means[k]) +
                                                          3.0f * std::log(2.0f * static_cast<float>(M_PI))));
            ll += w[i] * logsumexpN(logs);
        }
        float p = 7.0f * K - 1.0f; // parameters count for K components (diag covars)
        float bic = -2.0f * ll + p * std::log(std::max(1.0f, Neff));

        GMMResult r; r.K = K; r.means = means; r.variances = vars; r.weights = pis;
        return {bic, r};
    };

    if (opts.force_K >= 1 && opts.force_K <= 3) {
        auto [bic, res] = em_for_K(opts.force_K);
        // ignore BIC value when forcing K; return the result directly
        return res;
    } else {
        float best_bic = std::numeric_limits<float>::infinity();
        for (int K : {1,2,3}) {
            auto [bic, res] = em_for_K(K);
            // Apply extra penalty per additional component to bias towards simpler models when visually single color
            float adj_bic = bic + std::max(0.0f, opts.bic_extra_per_component) * static_cast<float>(K - 1);
            if (adj_bic < best_bic) { best_bic = adj_bic; best = std::move(res); }
        }
        return best;
    }
}

std::vector<int> GMMResult::predict(const std::vector<Eigen::Vector3f>& X) const {
    std::vector<int> labels; labels.reserve(X.size());
    if (K <= 0) return labels;
    const float var_floor = 25.0f;
    for (const auto& x : X) {
        int best_k = 0; float best_ll = -std::numeric_limits<float>::infinity();
        for (int k = 0; k < K; ++k) {
            Eigen::Vector3f vv = variances[k].cwiseMax(var_floor);
            Eigen::Vector3f d = x - means[k];
            float log_det = std::log(vv[0]) + std::log(vv[1]) + std::log(vv[2]);
            float quad = d[0]*d[0]/vv[0] + d[1]*d[1]/vv[1] + d[2]*d[2]/vv[2];
            float ll = std::log(std::max(1e-8f, weights[k])) - 0.5f*(log_det + quad + 3.0f*std::log(2.0f*(float)M_PI));
            if (ll > best_ll) { best_ll = ll; best_k = k; }
        }
        labels.push_back(best_k);
    }
    return labels;
}

} // namespace pcg
