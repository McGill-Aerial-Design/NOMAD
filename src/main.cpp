// SPDX-License-Identifier: Apache-2.0
// Production CLI composition root: parse one typed request and send it to the
// persistent NOMAD runtime.
#include "cli_command_table.hpp"
#include "cli_commands.hpp"
#include "runtime/cli_client.hpp"

#include <cstdlib>

int main(int argc, char **argv) {
    const auto arguments = parse_arguments(argc, argv);
    if (!arguments.has_value()) {
        print_usage();
        return EXIT_FAILURE;
    }
    return run_runtime_command(*arguments);
}
