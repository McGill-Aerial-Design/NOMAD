# Reuse the reviewed superbuild download; NOMAD owns its JSON version and consumers.
# MAVSDK's configure has already installed the dependency's CMake package.
find_package(nlohmann_json 3.12.0 EXACT CONFIG REQUIRED
    PATHS "${NOMAD_MAVSDK_INSTALL_DIR}" NO_DEFAULT_PATH)
