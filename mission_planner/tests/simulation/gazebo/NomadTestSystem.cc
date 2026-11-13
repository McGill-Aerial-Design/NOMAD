// SPDX-License-Identifier: Apache-2.0
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Camera.hh>
#include <sdf/Camera.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>

namespace nomad_test
{
class NomadTestSystem final : public gz::sim::System, public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
    struct Pose
    {
        int64_t ns;
        gz::math::Pose3d camera, occluder, body, selected, lookalike;
        gz::math::AxisAlignedBox bounds;
        bool haveOccluder{false};
        int64_t motionElapsed{-1};
    };
    struct Image { gz::msgs::Image value; int64_t received; };
    std::array<gz::sim::Joint, 3> joints;
    gz::sim::Entity camera{gz::sim::kNullEntity};
    unsigned int imageWidth{640}, imageHeight{480};
    double imageHfov{M_PI / 2}, motionDuration{0};
    std::array<std::deque<std::pair<int64_t, double>>, 3> commands;
    std::array<double, 3> target{0, 0, 0};
    const std::array<double, 3> lower{-M_PI / 6, -M_PI / 4, -160 * M_PI / 180};
    const std::array<double, 3> upper{M_PI / 6, 3 * M_PI / 4, 160 * M_PI / 180};
    const std::array<double, 3> rates{20 * M_PI / 180, 30 * M_PI / 180, 30 * M_PI / 180};
    std::deque<Pose> poses;
    std::deque<Image> images;
    std::deque<Image> masks;
    std::mutex gate;
    std::condition_variable changed;
    std::atomic<int64_t> now{0};
    std::atomic<uint64_t> droppedImages{0};
    std::atomic<bool> running{true};
    std::thread writer;
    std::filesystem::path output;
    std::ofstream truth;
    // Transport is destroyed before the state captured by its callbacks.
    gz::transport::Node node;
    gz::transport::Node::Publisher streaming;
    gz::transport::Node::Publisher encodedImages;
    bool enabled{false};
    bool boundedPngs{false};
    unsigned int savedPngs{0};
    bool occlusion{false};
    gz::sim::Entity world{gz::sim::kNullEntity};
    gz::sim::Model occluder;
    gz::sim::Link occluderBody;
    int64_t occlusionStart{-1}, nextPoll{0};
    std::optional<gz::math::AxisAlignedBox> observedBounds;
    bool boundsDisabled{false};
    std::string objectMotion;
    gz::sim::Link aircraftBody;
    double motionSpeed{5};
    int64_t motionStart{-1}, motionNextPoll{0};
    bool motionComplete{false};
    std::string targetCase{"static"};
    gz::sim::Model selectedDeer, otherDeer, objectOccluder;
    int64_t nextTargetUpdate{0};
    bool targetCovered{false};

    void MoveTargets(double elapsed, gz::sim::EntityComponentManager &ecm)
    {
        if (targetCase == "static" || elapsed < 0) { return; }
        // Visual actors need camera-rate poses, not 1,000 expensive scene teleports per simulation second.
        if (now < nextTargetUpdate) { return; }
        nextTargetUpdate = now + 33333333;
        if (selectedDeer.Entity() == gz::sim::kNullEntity)
        {
            world = gz::sim::worldEntity(ecm);
            const gz::sim::World scene(world);
            selectedDeer = gz::sim::Model(scene.ModelByName(ecm, "selected_deer"));
            otherDeer = gz::sim::Model(scene.ModelByName(ecm, "other_deer"));
            objectOccluder = gz::sim::Model(scene.ModelByName(ecm, "object_occluder"));
            if (selectedDeer.Entity() == gz::sim::kNullEntity || otherDeer.Entity() == gz::sim::kNullEntity ||
                objectOccluder.Entity() == gz::sim::kNullEntity) { throw std::runtime_error("Target fixture missing"); }
        }
        const double t = std::min(elapsed, motionDuration);
        const gz::math::Vector3d selected(2 * sin(2 * M_PI * t / 12), sin(2 * M_PI * t / 8), .015);
        selectedDeer.SetWorldPoseCmd(ecm, gz::math::Pose3d(selected, {0, 0, .2 * sin(2 * M_PI * t / 12)}));
        if (targetCase == "occlusion" && t >= motionDuration * .3 && t <= motionDuration * .55)
        {
            const auto body = gz::sim::worldPose(aircraftBody.Entity(), ecm).Pos();
            auto blocker = selected + .2 * (body - selected); blocker.Z(2);
            objectOccluder.SetWorldPoseCmd(ecm, gz::math::Pose3d(blocker, {0, 0, 0}));
            targetCovered = true;
        }
        else if (targetCovered)
        { objectOccluder.SetWorldPoseCmd(ecm, {30, 30, 2, 0, 0, 0}); targetCovered = false; }
        if (targetCase == "lookalike")
        {
            const double fraction = std::clamp((t - motionDuration * .2) / (motionDuration * .4), 0.0, 1.0);
            const gz::math::Vector3d other = (1 - fraction) * gz::math::Vector3d(6, 8, .015) +
                fraction * (selected + gz::math::Vector3d(-2.5, 0, 0));
            otherDeer.SetWorldPoseCmd(ecm, gz::math::Pose3d(other, {0, 0, .2 * sin(2 * M_PI * t / 12)}));
        }
    }

    void MoveAircraft(gz::sim::EntityComponentManager &ecm)
    {
        if (objectMotion.empty()) { return; }
        if (motionStart < 0 && now >= motionNextPoll)
        {
            motionNextPoll = now + 100000000;
            std::ifstream request(output / "object-motion.start");
            std::string owner; std::getline(request, owner);
            if (owner == output.filename().string())
            {
                motionStart = now;
                const auto path = output / "object-motion-ack.json";
                std::ofstream(path.string() + ".tmp") << "{\"start_sim_ns\":" << motionStart << "}\n";
                std::filesystem::rename(path.string() + ".tmp", path);
            }
        }
        const double duration = motionDuration;
        const double t = motionStart < 0 ? -1 : (now - motionStart) / 1e9;
        gz::math::Vector3d velocity{0, 0, 0}, angular{0, 0, 0};
        if (t >= 0 && t < duration)
        {
            const double amplitude = objectMotion == "orbit" ? 10 : objectMotion == "approach" ? 5 : 0;
            const double deviation = amplitude * M_PI / 180 * sin(2 * M_PI * t / 8);
            velocity = {motionSpeed * cos(deviation), -motionSpeed * sin(deviation), 0};
            angular.Z() = (objectMotion == "orbit" ? motionSpeed / 20 : 0) +
                amplitude * M_PI / 180 * (2 * M_PI / 8) * cos(2 * M_PI * t / 8);
        }
        else if (t >= duration && !motionComplete)
        {
            const auto path = output / "object-motion-complete.json";
            std::ofstream(path.string() + ".tmp") << "{\"end_sim_ns\":" << now << "}\n";
            std::filesystem::rename(path.string() + ".tmp", path);
            motionComplete = true;
        }
        // This is an independent kinematic BODY disturbance, not autopilot flight dynamics or camera targeting.
        aircraftBody.SetLinearVelocity(ecm, velocity);
        aircraftBody.SetAngularVelocity(ecm, angular);
        MoveTargets(t, ecm);
    }

    void MoveOccluder(gz::sim::EntityComponentManager &ecm)
    {
        if (!occlusion) { return; }
        if (occluder.Entity() == gz::sim::kNullEntity)
        {
            world = gz::sim::worldEntity(ecm);
            occluder = gz::sim::Model(gz::sim::World(world).ModelByName(ecm, "test_occluder"));
            if (occluder.Entity() == gz::sim::kNullEntity)
            {
                if (now < 1000000000) { return; }
                throw std::runtime_error("Test occluder missing in world " + std::to_string(world));
            }
            occluderBody = gz::sim::Link(occluder.LinkByName(ecm, "body"));
            if (occluderBody.Entity() == gz::sim::kNullEntity) { throw std::runtime_error("Occluder body missing"); }
            occluderBody.EnableBoundingBoxChecks(ecm);
        }
        // This fixture changes pose only; its observed local collision geometry stays fixed.
        if (observedBounds && !boundsDisabled)
        { occluderBody.EnableBoundingBoxChecks(ecm, false); boundsDisabled = true; }
        if (occlusionStart < 0 && now >= nextPoll)
        {
            nextPoll = now + 100000000;
            std::ifstream request(output / "occluder.start");
            std::string owner; std::getline(request, owner);
            if (owner == output.filename().string())
            {
                occlusionStart = now;
                auto path = output / "occluder-ack.json";
                std::ofstream(path.string() + ".tmp") << "{\"request_observed_sim_ns\":" << occlusionStart << "}\n";
                std::filesystem::rename(path.string() + ".tmp", path);
            }
        }
        if (occlusionStart < 0) { return; }
        const double t = (now - occlusionStart) / 1e9;
        const double offset = t < 1 ? 12 : t < 2 ? 12 * (2 - t) : t < 3.5 ? 0 : t < 4.5 ? 12 * (t - 3.5) : 12;
        occluder.SetWorldPoseCmd(ecm, gz::math::Pose3d(6, -3 + offset, 6, 0, 0, 0));
    }

    gz::sim::Entity Find(const std::string &name, const gz::sim::EntityComponentManager &ecm,
        gz::sim::Entity model)
    {
        const auto found = gz::sim::entitiesFromScopedName(name, ecm, model);
        if (found.size() != 1) { throw std::runtime_error("Expected one entity: " + name); }
        return *found.begin();
    }

    void Command(size_t axis, double value)
    {
        if (!running || !std::isfinite(value)) { return; }
        std::lock_guard<std::mutex> lock(gate);
        commands[axis].emplace_back(now.load(), std::clamp(value, lower[axis], upper[axis]));
        if (commands[axis].size() > 2048)
        { std::ofstream(output / "capture-error.txt") << "Mount command queue exceeded bound\n"; running = false; }
    }

    void OnImage(const gz::msgs::Image &value)
    {
        if (!running) { return; }
        uint64_t sequence = 0; bool identified = false;
        for (const auto &data : value.header().data())
        {
            if (data.key() == "seq" && data.value_size() == 1)
            { sequence = std::stoull(data.value(0)); identified = true; }
        }
        if (!identified || sequence > 0xffffff || value.width() < 64 || value.height() < 64 ||
            value.step() != value.width() * 3 || value.data().size() != size_t(value.width()) * value.height() * 3)
        { std::ofstream(output / "capture-error.txt") << "Invalid capture stamp input\n"; return; }
        // Test-only timecode, decoded and erased before NOMAD sees pixels; no truth labels or poses are encoded.
        auto encoded = value;
        auto &pixels = *encoded.mutable_data();
        const uint64_t stamp = 0xa65cULL | (sequence << 16) | ((~sequence & 0xffffffULL) << 40);
        for (unsigned y = 0; y < 64; ++y) for (unsigned x = 0; x < 64; ++x)
        {
            const char level = ((stamp >> ((y / 8) * 8 + x / 8)) & 1) ? char(255) : char(0);
            for (unsigned channel = 0; channel < 3; ++channel) { pixels[y * value.step() + x * 3 + channel] = level; }
        }
        encodedImages.Publish(encoded);
        const auto received = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        std::lock_guard<std::mutex> lock(gate);
        if (images.size() == 16) { images.pop_front(); ++droppedImages; }
        images.push_back({value, received});
        changed.notify_one();
    }

    void OnMask(const gz::msgs::Image &value)
    {
        std::lock_guard<std::mutex> lock(gate);
        if (masks.size() == 32) { masks.pop_front(); }
        masks.push_back({value, 0}); changed.notify_one();
    }

    void WriteImages()
    {
        try
        {
            while (running)
            {
                Image image;
                std::optional<gz::msgs::Image> mask;
                Pose pose;
                int64_t stamp;
                {
                    std::unique_lock<std::mutex> lock(gate);
                    changed.wait_for(lock, std::chrono::milliseconds(100), [this] { return !images.empty() || !running; });
                    if (images.empty() || poses.empty()) { continue; }
                    const auto &header = images.front().value.header();
                    stamp = header.stamp().sec() * 1000000000LL + header.stamp().nsec();
                    auto nearest = std::min_element(poses.begin(), poses.end(), [stamp](const Pose &a, const Pose &b) {
                        return std::abs(a.ns - stamp) < std::abs(b.ns - stamp);
                    });
                    if (std::abs(nearest->ns - stamp) > 2000000)
                    {
                        if (poses.back().ns < stamp)
                        { changed.wait_for(lock, std::chrono::milliseconds(1)); continue; }
                        throw std::runtime_error("Capture pose alignment exceeds 2 ms");
                    }
                    pose = *nearest; image = std::move(images.front()); images.pop_front();
                    if (!objectMotion.empty())
                    {
                        auto maskStamp = [](const Image &item) {
                            return item.value.header().stamp().sec() * 1000000000LL + item.value.header().stamp().nsec();
                        };
                        auto matched = std::min_element(masks.begin(), masks.end(), [&](const Image &a, const Image &b) {
                            return std::abs(maskStamp(a) - stamp) < std::abs(maskStamp(b) - stamp);
                        });
                        if (matched == masks.end() || std::abs(maskStamp(*matched) - stamp) > 2000000)
                        {
                            const auto hostNow = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch()).count();
                            if (hostNow - image.received > 2000000000)
                            { throw std::runtime_error("Semantic mask/capture alignment deadline (2 ms sim /2 s host)"); }
                            images.push_front(std::move(image));
                            changed.wait_for(lock, std::chrono::milliseconds(2)); continue;
                        }
                        mask = matched->value;
                    }
                }
                uint64_t sequence = 0;
                bool haveSequence = false;
                for (const auto &data : image.value.header().data())
                {
                    if (data.key() == "seq" && data.value_size() == 1)
                    { sequence = std::stoull(data.value(0)); haveSequence = true; }
                }
                if (!haveSequence || image.value.width() != imageWidth || image.value.height() != imageHeight ||
                    image.value.step() != imageWidth * 3 || image.value.data().size() != size_t(imageWidth) * imageHeight * 3 ||
                    image.value.pixel_format_type() != gz::msgs::RGB_INT8)
                { throw std::runtime_error("Unexpected capture identity/format"); }
                cv::Mat rgb(imageHeight, imageWidth, CV_8UC3,
                    const_cast<char *>(image.value.data().data()), imageWidth * 3);
                std::ostringstream name;
                name << "capture_" << std::setw(8) << std::setfill('0') << sequence;
                const bool tail = !objectMotion.empty() && pose.motionElapsed >= (motionDuration - .5) * 1e9 &&
                    pose.motionElapsed <= (motionDuration + .5) * 1e9 && sequence % 6 == 0;
                const bool savePng = !boundedPngs || (savedPngs < 120 &&
                    (sequence < 60 || sequence % 600 == 0 || tail));
                if (savePng)
                {
                    cv::Mat bgr; cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
                    if (!cv::imwrite((output / "frames" / (name.str() + ".png")).string(), bgr,
                        {cv::IMWRITE_PNG_COMPRESSION, 1}))
                    { throw std::runtime_error("Capture PNG write failed"); }
                    ++savedPngs;
                }
                std::string fingerprint;
                const char *hex = "0123456789abcdef";
                for (int y = 0; y < 48; ++y) for (int x = 0; x < 64; ++x)
                {
                    const auto pixel = rgb.at<cv::Vec3b>((y * 10 + 5) * imageHeight / 480,
                        (x * 10 + 5) * imageWidth / 640);
                    const int gray = (77 * pixel[0] + 150 * pixel[1] + 29 * pixel[2]) >> 8;
                    fingerprint += hex[gray >> 4]; fingerprint += hex[gray & 15];
                }
                auto path = output / "frames" / (name.str() + ".json");
                std::ofstream metadata(path.string() + ".tmp");
                metadata << std::setprecision(17) << "{\"sequence\":" << sequence << ",\"capture_sim_ns\":" << stamp
                    << ",\"width\":" << imageWidth << ",\"height\":" << imageHeight << ",\"hfov_radians\":" << imageHfov
                    << ",\"raw_queue_dropped\":" << droppedImages.load()
                    << ",\"raw_received_linux_ns\":" << image.received << ",\"pose_delta_ns\":" << pose.ns - stamp
                    << ",\"position\":[" << pose.camera.Pos().X() << ',' << pose.camera.Pos().Y() << ',' << pose.camera.Pos().Z()
                    << "],\"quaternion\":[" << pose.camera.Rot().W() << ',' << pose.camera.Rot().X() << ','
                    << pose.camera.Rot().Y() << ',' << pose.camera.Rot().Z() << "],\"fingerprint\":\"" << fingerprint << '"'
                    << ",\"png_recorded\":" << (savePng ? "true" : "false");
                if (mask)
                {
                    if (mask->width() != imageWidth || mask->height() != imageHeight || mask->step() != imageWidth * 3 ||
                        mask->data().size() != size_t(imageWidth) * imageHeight * 3 ||
                        mask->pixel_format_type() != gz::msgs::RGB_INT8)
                    { throw std::runtime_error("Unexpected semantic mask format"); }
                    const std::string maskName = name.str() + ".mask.png";
                    cv::Mat labels(imageHeight, imageWidth, CV_8UC3,
                        const_cast<char *>(mask->data().data()), imageWidth * 3), channels[3];
                    cv::split(labels, channels);
                    if (cv::norm(channels[0], channels[1], cv::NORM_INF) != 0 ||
                        cv::norm(channels[0], channels[2], cv::NORM_INF) != 0)
                    { throw std::runtime_error("Semantic label channels differ"); }
                    if (!cv::imwrite((output / "frames" / maskName).string(), channels[0],
                        {cv::IMWRITE_PNG_COMPRESSION, 1}))
                    { throw std::runtime_error("Semantic mask write failed"); }
                    const int64_t maskNs = mask->header().stamp().sec() * 1000000000LL + mask->header().stamp().nsec();
                    metadata << ",\"mask\":\"" << maskName << "\",\"mask_delta_ns\":" << maskNs - stamp
                        << ",\"motion_elapsed_sim_ns\":" << pose.motionElapsed << ",\"body_position\":["
                        << pose.body.Pos().X() << ',' << pose.body.Pos().Y() << ',' << pose.body.Pos().Z()
                        << "],\"body_quaternion\":[" << pose.body.Rot().W() << ',' << pose.body.Rot().X() << ','
                        << pose.body.Rot().Y() << ',' << pose.body.Rot().Z() << ']';
                    if (targetCase != "static" && pose.motionElapsed >= 0)
                    {
                        metadata << ",\"selected_position\":[" << pose.selected.Pos().X() << ','
                            << pose.selected.Pos().Y() << ',' << pose.selected.Pos().Z() << ']'
                            << ",\"lookalike_position\":[" << pose.lookalike.Pos().X() << ','
                            << pose.lookalike.Pos().Y() << ',' << pose.lookalike.Pos().Z() << ']';
                    }
                }
                if (pose.haveOccluder)
                {
                    metadata << ",\"occluder\":{\"position\":[" << pose.occluder.Pos().X() << ','
                        << pose.occluder.Pos().Y() << ',' << pose.occluder.Pos().Z() << "],\"quaternion\":["
                        << pose.occluder.Rot().W() << ',' << pose.occluder.Rot().X() << ',' << pose.occluder.Rot().Y()
                        << ',' << pose.occluder.Rot().Z() << "],\"min\":[" << pose.bounds.Min().X() << ','
                        << pose.bounds.Min().Y() << ',' << pose.bounds.Min().Z() << "],\"max\":[" << pose.bounds.Max().X()
                        << ',' << pose.bounds.Max().Y() << ',' << pose.bounds.Max().Z() << "]}";
                }
                metadata << "}\n";
                metadata.close();
                if (!metadata) { throw std::runtime_error("Capture metadata write failed"); }
                std::filesystem::rename(path.string() + ".tmp", path);
            }
        }
        catch (const std::exception &error)
        { std::ofstream(output / "capture-error.txt") << error.what() << '\n'; }
    }

public:
    ~NomadTestSystem() override
    {
        running = false;
        for (const auto &axis : {"roll", "pitch", "yaw"}) { node.Unsubscribe(std::string("/gimbal/cmd_") + axis); }
        node.Unsubscribe("/nomad/rgb");
        node.Unsubscribe("/nomad/oracle/semantic/labels_map");
        changed.notify_one(); if (writer.joinable()) { writer.join(); }
    }

    void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
        gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
    {
        output = sdf->Get<std::string>("output");
        boundedPngs = sdf->HasElement("bounded_pngs") && sdf->Get<bool>("bounded_pngs");
        occlusion = sdf->HasElement("occlusion") && sdf->Get<bool>("occlusion");
        if (sdf->HasElement("object_motion"))
        {
            objectMotion = sdf->Get<std::string>("object_motion");
            if (sdf->HasElement("target_case")) { targetCase = sdf->Get<std::string>("target_case"); }
            motionSpeed = sdf->Get<double>("motion_speed");
            motionDuration = objectMotion == "orbit" ? 2 * M_PI * 20 / motionSpeed :
                (objectMotion == "approach" ? 27 : 60) / motionSpeed;
            aircraftBody = gz::sim::Link(Find("iris_with_standoffs::base_link", ecm, entity));
            node.Subscribe("/nomad/oracle/semantic/labels_map", &NomadTestSystem::OnMask, this);
        }
        world = gz::sim::worldEntity(entity, ecm);
        std::filesystem::create_directory(output / "frames");
        truth.open(output / "joints.csv");
        truth << "simulation_ns,roll,pitch,yaw,roll_velocity,pitch_velocity,yaw_velocity\n";
        const std::array<std::string, 3> axes{"roll", "pitch", "yaw"};
        for (size_t axis = 0; axis < 3; ++axis)
        {
            joints[axis] = gz::sim::Joint(Find("gimbal::" + axes[axis] + "_joint", ecm, entity));
            joints[axis].EnablePositionCheck(ecm);
            joints[axis].EnableVelocityCheck(ecm);
            joints[axis].SetPositionLimits(ecm, {{lower[axis], upper[axis]}});
            joints[axis].SetVelocityLimits(ecm, {{-rates[axis], rates[axis]}});
            joints[axis].SetEffortLimits(ecm, {{-1000000, 1000000}});
            node.Subscribe<gz::msgs::Double>("/gimbal/cmd_" + axes[axis],
                [this, axis](const gz::msgs::Double &message) { Command(axis, message.data()); });
        }
        camera = Find("gimbal::pitch_link::camera", ecm, entity);
        const auto sensor = ecm.Component<gz::sim::components::Camera>(camera);
        if (!sensor || !sensor->Data().CameraSensor()) { throw std::runtime_error("Camera intrinsics missing"); }
        imageWidth = sensor->Data().CameraSensor()->ImageWidth();
        imageHeight = sensor->Data().CameraSensor()->ImageHeight();
        imageHfov = sensor->Data().CameraSensor()->HorizontalFov().Radian();
        if (imageWidth < 16 || imageWidth > 1920 || imageHeight < 16 || imageHeight > 1080 || !std::isfinite(imageHfov))
        { throw std::runtime_error("Camera geometry outside owned fixture budget"); }
        encodedImages = node.Advertise<gz::msgs::Image>("/nomad/encoded_rgb");
        node.Subscribe("/nomad/rgb", &NomadTestSystem::OnImage, this);
        streaming = node.Advertise<gz::msgs::Boolean>("/nomad/rgb/enable_streaming");
        writer = std::thread([this] { WriteImages(); });
    }

    void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
    {
        if (info.paused) { return; }
        if (!enabled && streaming.HasConnections())
        {
            gz::msgs::Boolean message; message.set_data(true);
            enabled = streaming.Publish(message);
            std::ofstream(output / "stream-enable.txt") << (enabled ? "published" : "failed") << '\n';
        }
        now = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
        MoveOccluder(ecm);
        MoveAircraft(ecm);
        std::lock_guard<std::mutex> lock(gate);
        for (size_t axis = 0; axis < 3; ++axis)
        {
            auto &queue = commands[axis];
            while (!queue.empty() && queue.front().first <= now - 100000000)
            { target[axis] = queue.front().second; queue.pop_front(); }
            const auto position = joints[axis].Position(ecm);
            if (!position || position->empty()) { continue; }
            const double actual = (*position)[0];
            double velocity = std::clamp((target[axis] - actual) / .080, -rates[axis], rates[axis]);
            if ((actual <= lower[axis] && velocity < 0) || (actual >= upper[axis] && velocity > 0)) { velocity = 0; }
            joints[axis].SetVelocity(ecm, {velocity});
        }
    }

    void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override
    {
        if (info.paused) { return; }
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
        {
            std::lock_guard<std::mutex> lock(gate);
            Pose pose{ns, gz::sim::worldPose(camera, ecm)};
            if (!objectMotion.empty())
            { pose.body = gz::sim::worldPose(aircraftBody.Entity(), ecm); pose.motionElapsed = motionStart < 0 ? -1 : ns - motionStart; }
            if (selectedDeer.Entity() != gz::sim::kNullEntity)
            {
                pose.selected = gz::sim::worldPose(selectedDeer.Entity(), ecm);
                pose.lookalike = gz::sim::worldPose(otherDeer.Entity(), ecm);
            }
            if (occlusion && occluderBody.Entity() != gz::sim::kNullEntity)
            {
                if (!observedBounds)
                {
                    const auto bounds = occluderBody.AxisAlignedBox(ecm);
                    if (bounds && bounds->Min().IsFinite() && bounds->Max().IsFinite() && bounds->Volume() > 0)
                    { observedBounds = *bounds; }
                }
                if (observedBounds)
                { pose.occluder = gz::sim::worldPose(occluderBody.Entity(), ecm); pose.bounds = *observedBounds; pose.haveOccluder = true; }
            }
            poses.push_back(pose);
            if (poses.size() > 2048) { poses.pop_front(); }
        }
        if (ns % 10000000 != 0) { return; }
        truth << ns;
        for (auto &joint : joints) { const auto value = joint.Position(ecm); truth << ',' << (value && !value->empty() ? (*value)[0] : NAN); }
        for (auto &joint : joints) { const auto value = joint.Velocity(ecm); truth << ',' << (value && !value->empty() ? (*value)[0] : NAN); }
        truth << '\n'; truth.flush();
        if (!truth)
        { std::ofstream(output / "capture-error.txt") << "Joint truth write failed\n"; running = false; }
    }
};
}
GZ_ADD_PLUGIN(nomad_test::NomadTestSystem, gz::sim::System,
    nomad_test::NomadTestSystem::ISystemConfigure, nomad_test::NomadTestSystem::ISystemPreUpdate,
    nomad_test::NomadTestSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(nomad_test::NomadTestSystem, "NomadTestSystem")
