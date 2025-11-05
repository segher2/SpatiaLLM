#include "pcg/csv_writer.hpp"

#include <fstream>
#include <sstream>

namespace pcg {

struct CsvWriter::Impl {
    std::ofstream os;
    bool wrote_header = false;
};

CsvWriter::CsvWriter(const std::string& path) : impl_(new Impl{}) {
    impl_->os.open(path);
}

bool CsvWriter::good() const { return impl_ && impl_->os.good(); }

static std::string escape_csv(const std::string& s) {
    bool need_quote = s.find_first_of(",\n\r\"") != std::string::npos;
    if (!need_quote) return s;
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += '"';
        out += c;
    }
    out += '"';
    return out;
}

void CsvWriter::writeHeader(const std::vector<std::string>& headers) {
    if (!impl_->os) return;
    for (std::size_t i = 0; i < headers.size(); ++i) {
        if (i) impl_->os << ',';
        impl_->os << escape_csv(headers[i]);
    }
    impl_->os << '\n';
    impl_->wrote_header = true;
}

void CsvWriter::writeRow(const std::vector<std::string>& cols) {
    if (!impl_->os) return;
    for (std::size_t i = 0; i < cols.size(); ++i) {
        if (i) impl_->os << ',';
        impl_->os << escape_csv(cols[i]);
    }
    impl_->os << '\n';
}

CsvWriter::~CsvWriter() { delete impl_; }

} // namespace pcg
