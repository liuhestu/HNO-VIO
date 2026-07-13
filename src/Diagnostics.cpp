#include "hno_vio/Diagnostics.h"

#include <iostream>

namespace hno_vio {

void Diagnostics::printPipeline(const PipelineDiagnostics& diagnostics) const {
    std::cout << "[Pipeline] t=" << diagnostics.timestamp
              << " initialized=" << (diagnostics.initialized ? "true" : "false")
              << " stage=" << diagnostics.stage << std::endl;
}

} // namespace hno_vio
