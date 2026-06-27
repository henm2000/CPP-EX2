#pragma once

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>

namespace drone_mapper::parse_utils {

inline std::string trim(std::string s) {
    const auto notspace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
    return s;
}

inline std::optional<double> tryDouble(const std::string& s) {
    if (s.empty()) return std::nullopt;
    try {
        std::size_t pos = 0;
        const double val = std::stod(s, &pos);
        while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos]))) ++pos;
        if (pos != s.size()) return std::nullopt;
        return val;
    } catch (...) {
        return std::nullopt;
    }
}

} // namespace drone_mapper::parse_utils
