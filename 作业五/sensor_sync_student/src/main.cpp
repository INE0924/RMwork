#include <atomic>
#include <charconv>
#include <cctype>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "sensor_sync.hpp"
#include "thread_safe_queue.hpp"

namespace fs = std::filesystem;

namespace {

struct Config {
  std::size_t queue_capacity;
  std::uint64_t max_delta_us;
  fs::path camera_file;
  fs::path imu_file;
  fs::path output_file;
};

std::string trim(const std::string& value) {
  const std::string whitespace = " \t\r\n\v\f";
  const auto first = value.find_first_not_of(whitespace);
  if (first == std::string::npos) return "";
  const auto last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1);
}

std::uint64_t parse_unsigned(const std::string& text,
                             const std::string& field) {
  std::uint64_t value = 0;
  const char* first = text.data();
  const char* last = first + text.size();
  const auto parsed = std::from_chars(first, last, value, 10);
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != last) {
    throw std::runtime_error("invalid non-negative integer for " + field +
                             ": '" + text + "'");
  }
  return value;
}

Config read_config(const fs::path& config_file) {
  std::ifstream in(config_file);
  if (!in.is_open()) {
    throw std::runtime_error("cannot open config file: " + config_file.string());
  }

  std::unordered_map<std::string, std::string> values;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(in, line)) {
    ++line_number;
    line = trim(line);
    if (line.empty() || line[0] == '#') continue;
    const auto equal = line.find('=');
    if (equal == std::string::npos) {
      throw std::runtime_error("invalid config line " +
                               std::to_string(line_number) + ": missing '='");
    }
    const std::string key = trim(line.substr(0, equal));
    const std::string value = trim(line.substr(equal + 1));
    if (key != "queue_capacity" && key != "max_delta_us" &&
        key != "camera_file" && key != "imu_file" &&
        key != "output_file") {
      throw std::runtime_error("unknown config key on line " +
                               std::to_string(line_number) + ": " + key);
    }
    if (value.empty()) {
      throw std::runtime_error("empty config value for " + key);
    }
    if (!values.emplace(key, value).second) {
      throw std::runtime_error("duplicate config key: " + key);
    }
  }
  if (in.bad()) throw std::runtime_error("error while reading config file");

  const auto required = [&](const std::string& key) -> const std::string& {
    const auto it = values.find(key);
    if (it == values.end()) throw std::runtime_error("missing config key: " + key);
    return it->second;
  };

  const auto capacity = parse_unsigned(required("queue_capacity"), "queue_capacity");
  if (capacity == 0 || capacity > std::numeric_limits<std::size_t>::max()) {
    throw std::runtime_error("queue_capacity must be positive and fit size_t");
  }

  const fs::path base = fs::absolute(config_file).parent_path();
  const auto resolve = [&](const std::string& key) {
    const fs::path path(required(key));
    return path.is_absolute() ? path : (base / path).lexically_normal();
  };

  return {static_cast<std::size_t>(capacity),
          parse_unsigned(required("max_delta_us"), "max_delta_us"),
          resolve("camera_file"), resolve("imu_file"), resolve("output_file")};
}

// The producer checks that each file has monotonic timestamps, as specified.
// A false return from queue.push() means an error in another worker closed it.
void produce(const fs::path& path, SensorType type,
             ThreadSafeQueue<SensorData>& queue) {
  std::ifstream in(path);
  if (!in.is_open()) {
    throw std::runtime_error("cannot open input file: " + path.string());
  }

  std::string line;
  std::size_t line_number = 0;
  std::uint64_t previous_timestamp = 0;
  bool has_previous = false;

  while (std::getline(in, line)) {
    ++line_number;
    std::istringstream row(line);
    std::string seq_text, timestamp_text, extra;
    if (!(row >> seq_text >> timestamp_text) || (row >> extra)) {
      throw std::runtime_error(path.string() + ": line " +
                               std::to_string(line_number) +
                               ": expected exactly 'sequence timestamp_us'");
    }

    const std::string location = path.string() + ": line " +
                                 std::to_string(line_number);
    const auto sequence = parse_unsigned(seq_text, location + " sequence");
    const auto timestamp = parse_unsigned(timestamp_text, location + " timestamp_us");
    if (sequence > std::numeric_limits<std::uint32_t>::max()) {
      throw std::runtime_error(location + ": sequence exceeds uint32 range");
    }
    if (has_previous && timestamp < previous_timestamp) {
      throw std::runtime_error(location + ": timestamps are not nondecreasing");
    }
    previous_timestamp = timestamp;
    has_previous = true;
    if (!queue.push({type, static_cast<std::uint32_t>(sequence), timestamp})) {
      return;  // Queue was closed by an error in another thread.
    }
  }
  if (in.bad()) throw std::runtime_error("error reading input file: " + path.string());
}

void write_results(const fs::path& path,
                   const std::vector<SyncResult>& results) {
  std::ofstream out(path, std::ios::trunc);
  if (!out.is_open()) {
    throw std::runtime_error("cannot open output file: " + path.string());
  }
  for (const auto& result : results) {
    out << "CAMERA " << result.camera.sequence;
    if (result.imu) {
      out << " IMU " << result.imu->sequence
          << " DELTA " << result.delta_us;
    } else {
      out << " UNMATCHED";
    }
    out << '\n';
  }
  out.close();
  if (!out) throw std::runtime_error("error writing output file: " + path.string());
}

void execute(const Config& config) {
  ThreadSafeQueue<SensorData> queue(config.queue_capacity);
  std::vector<SensorData> cameras;
  std::vector<SensorData> imus;
  std::atomic<int> remaining_producers{2};

  std::mutex error_mutex;
  std::exception_ptr first_error;
  const auto report_error = [&](std::exception_ptr error) {
    {
      std::lock_guard<std::mutex> lock(error_mutex);
      if (!first_error) first_error = error;
    }
    // Wake any producer blocked by a full queue and the consumer if empty.
    queue.close();
  };

  const auto producer_worker = [&](const fs::path& path, SensorType type) {
    try {
      produce(path, type, queue);
    } catch (...) {
      report_error(std::current_exception());
    }
    // Both normal completion and abnormal completion count as a finished producer.
    if (remaining_producers.fetch_sub(1) == 1) queue.close();
  };

  const auto consumer_worker = [&] {
    try {
      SensorData packet{};
      while (queue.pop(packet)) {
        if (packet.type == SensorType::CAMERA) {
          cameras.push_back(packet);
        } else {
          imus.push_back(packet);
        }
      }
    } catch (...) {
      report_error(std::current_exception());
    }
  };

  std::thread consumer_thread;
  std::thread camera_thread;
  std::thread imu_thread;
  try {
    consumer_thread = std::thread(consumer_worker);
    camera_thread = std::thread(producer_worker, config.camera_file, SensorType::CAMERA);
    imu_thread = std::thread(producer_worker, config.imu_file, SensorType::IMU);
  } catch (...) {
    queue.close();
    if (camera_thread.joinable()) camera_thread.join();
    if (imu_thread.joinable()) imu_thread.join();
    if (consumer_thread.joinable()) consumer_thread.join();
    throw;
  }

  camera_thread.join();
  imu_thread.join();
  consumer_thread.join();

  if (first_error) std::rethrow_exception(first_error);
  const auto results = synchronize(std::move(cameras), std::move(imus),
                                   config.max_delta_us);
  write_results(config.output_file, results);
  std::cout << "Processed " << results.size() << " camera packets; wrote "
            << config.output_file << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <config-file>\n";
    return 1;
  }
  try {
    execute(read_config(argv[1]));
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "sensor_sync error: " << error.what() << '\n';
    return 1;
  }
}
