#include "pcg/params.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cctype>

namespace pcg {

namespace {

inline void ltrim(std::string& s) { s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch){ return !std::isspace(ch); })); }
inline void rtrim(std::string& s) { s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end()); }
inline void trim(std::string& s) { ltrim(s); rtrim(s); }

inline bool starts_with(const std::string& s, const std::string& p) {
    return s.rfind(p, 0) == 0;
}

inline bool parse_kv_line(const std::string& line, std::string& key, std::string& value) {
    std::size_t pos = line.find(':');
    std::size_t pos_eq = line.find('=');
    std::size_t use = std::string::npos;
    if (pos != std::string::npos) use = pos;
    else if (pos_eq != std::string::npos) use = pos_eq;
    if (use == std::string::npos) return false;
    key = line.substr(0, use);
    value = line.substr(use + 1);
    trim(key); trim(value);
    // strip inline comments starting with '#'
    auto cpos = value.find('#');
    if (cpos != std::string::npos) {
        value = value.substr(0, cpos);
        trim(value);
    }
    return !key.empty();
}

} // namespace

bool load_params_from_file(const std::string& filepath, ParamsConfig& out, std::string* error) {
    std::ifstream ifs(filepath);
    if (!ifs) {
        if (error) *error = "Cannot open config: " + filepath;
        return false;
    }
    std::string line;
    while (std::getline(ifs, line)) {
        trim(line);
        if (line.empty()) continue;
        if (starts_with(line, "#")) continue;
        std::string key, value;
        if (!parse_kv_line(line, key, value)) continue;
        try {
            if (key == "radius") out.radius = std::stod(value);
            else if (key == "min_cluster_size") out.min_cluster_size = std::stoi(value);
            else if (key == "max_neighbors") out.max_neighbors = std::stoi(value);
            else if (key == "filter_factor") out.filter_factor = std::stod(value);
            else if (key == "no_filter_ratio") out.no_filter_ratio = std::stod(value);
            else if (key == "color_sample_n") out.color_sample_n = std::stoi(value);
            else if (key == "color_bic_k_penalty") out.color_bic_k_penalty = std::stod(value);
            else if (key == "color_min_weight") out.color_min_weight = std::stod(value);
            else if (key == "color_max_stddev") out.color_max_stddev = std::stod(value);
            else if (key == "color_deltaE_keep")  out.color_deltaE_keep  = std::stod(value);
            else if (key == "poisson_spacing_neighbors") out.poisson_spacing_neighbors = std::stoi(value);
            else if (key == "poisson_normal_neighbors")  out.poisson_normal_neighbors  = std::stoi(value);
            else if (key == "poisson_min_oriented_fraction") out.poisson_min_oriented_fraction = std::stod(value);
            else if (key == "poisson_require_closed") {
                std::string v = value; std::transform(v.begin(), v.end(), v.begin(), ::tolower);
                out.poisson_require_closed = (v == "1" || v == "true" || v == "yes");
            }
            else if (key == "poisson_invalid_ratio_vs_hull") out.poisson_invalid_ratio_vs_hull = std::stod(value);
            else if (key == "af_min_points") out.af_min_points = std::stoi(value);
            else if (key == "af_require_closed") {
                std::string v = value; std::transform(v.begin(), v.end(), v.begin(), ::tolower);
                out.af_require_closed = (v == "1" || v == "true" || v == "yes");
            }
            else if (key == "json_output") {
                std::string v = value; std::transform(v.begin(), v.end(), v.begin(), ::tolower);
                out.json_output = (v == "1" || v == "true" || v == "yes");
            }
        } catch (...) {
            // ignore individual parse errors
        }
    }
    return true;
}

} // namespace pcg
