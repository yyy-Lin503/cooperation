#ifndef TRAVELMATE_PROJECT_PATHS_HPP
#define TRAVELMATE_PROJECT_PATHS_HPP

#include <cstdlib>
#include <filesystem>
#include <string>

namespace RadarPaths {
namespace fs = std::filesystem;

inline std::string environment(const char* name) {
    const auto* value = std::getenv(name);
    return value ? value : "";
}

inline fs::path absolute(const fs::path& value, const fs::path& project) {
    return fs::weakly_canonical(value.is_absolute() ? value : project / value);
}

struct Paths {
    fs::path project, data, track;
    fs::path settings() const { return data / "config/gui_runtime.ini"; }
    fs::path radarRecordings() const { return data / "recordings/radar"; }
    fs::path cameraRecordings() const { return data / "recordings/camera"; }
    fs::path pointcloud() const { return data / "pointcloud"; }
    fs::path trackCsv() const { return data / "tracking/csv"; }
    fs::path diagnostics() const { return data / "diagnostics"; }
    fs::path reports() const { return data / "reports"; }
    fs::path radarConfig() const { return project / "config/radar"; }
    fs::path rosSource() const { return project / "ros2_ws/src/my_radar_follower"; }
    fs::path rosInstall() const { return project / "ros2_ws/install"; }

    void migrateSettings() const {
        fs::create_directories(settings().parent_path());
        const auto legacy = project / "gui_runtime.ini";
        if (fs::exists(legacy) && !fs::exists(settings()))
            fs::copy_file(legacy, settings(), fs::copy_options::skip_existing);
    }
};

inline Paths resolve(fs::path project = {}) {
    if (project.empty()) {
        const auto configured = environment("RADAR_PROJECT_ROOT");
        project = configured.empty() ? fs::current_path() : fs::u8path(configured);
    }
    project = fs::weakly_canonical(fs::absolute(project));
    const auto data = environment("RADAR_DATA_ROOT");
    const auto track = environment("RADAR_TRACK_DIR");
    const auto dataRoot = data.empty() ? project / "var" : absolute(fs::u8path(data), project);
    return {project, dataRoot, track.empty() ? dataRoot / "tracking/txt" : absolute(fs::u8path(track), project)};
}
} // namespace RadarPaths
#endif
