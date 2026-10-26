#include "project_paths.hpp"
#include <fstream>
#include <chrono>
#include <iostream>
#include <stdexcept>

static void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
static void env(const char* name, const std::string& value) {
#ifdef _WIN32
    _putenv_s(name, value.c_str());
#else
    if (value.empty()) unsetenv(name); else setenv(name, value.c_str(), 1);
#endif
}
int main() {
    namespace fs = std::filesystem;
    const auto root = fs::temp_directory_path() / ("travelmate-paths-" + std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count()));
    try {
        fs::create_directories(root);
        env("RADAR_DATA_ROOT", ""); env("RADAR_TRACK_DIR", "");
        auto paths = RadarPaths::resolve(root);
        check(paths.data == root / "var" && paths.track == root / "var/tracking/txt", "Default paths changed");
        env("RADAR_DATA_ROOT", "data with spaces"); env("RADAR_TRACK_DIR", "custom tracks");
        paths = RadarPaths::resolve(root);
        check(paths.data == root / "data with spaces" && paths.track == root / "custom tracks", "Relative overrides must use project root");
        env("RADAR_DATA_ROOT", (root / "external").u8string());
        env("RADAR_TRACK_DIR", (root / "external tracks").u8string());
        env("RADAR_PROJECT_ROOT", root.u8string());
        const auto oldCwd = fs::current_path(); fs::current_path(fs::temp_directory_path());
        paths = RadarPaths::resolve(); fs::current_path(oldCwd);
        check(paths.project == root && paths.track == root / "external tracks", "cwd changed overridden paths");
        const std::string original = "[General]\nmax_linear_speed_mps=0.9\nchassis_setup=/device/custom/setup.bash\n";
        std::ofstream(root / "gui_runtime.ini") << original;
        paths.migrateSettings();
        std::ifstream input(paths.settings());
        check(std::string(std::istreambuf_iterator<char>(input), {}) == original, "Configuration migration lost values");
        input.close(); std::ofstream(paths.settings()) << "saved settings";
        paths.migrateSettings();
        std::ifstream saved(paths.settings());
        check(std::string(std::istreambuf_iterator<char>(saved), {}) == "saved settings", "Repeated migration overwrote settings");
        saved.close(); fs::remove_all(root);
        std::cout << "PASS: path overrides, cwd independence and non-overwriting configuration migration\n";
        return 0;
    } catch (const std::exception& error) {
        fs::remove_all(root); std::cerr << error.what() << '\n'; return 1;
    }
}
