#pragma once

#include <string>
#include <vector>

namespace pcg {

struct CsvRow {
    std::vector<std::string> cols;
};

class CsvWriter {
public:
    explicit CsvWriter(const std::string& path);
    bool good() const;
    void writeHeader(const std::vector<std::string>& headers);
    void writeRow(const std::vector<std::string>& cols);
    ~CsvWriter();
private:
    struct Impl; Impl* impl_;
};

} // namespace pcg
