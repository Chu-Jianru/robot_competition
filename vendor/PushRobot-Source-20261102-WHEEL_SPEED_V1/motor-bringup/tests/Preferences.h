#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>

// 主机测试用 Preferences 假件：只在内存中保存命名空间键值，
// 数值统一以 double 存储，按真实 ESP32 Preferences 的取值/返回值语义工作。
// 另外支持按键的写入失败注入，用于验证 NVS 中途失败时的持久化顺序。
namespace fake_preferences {

struct Store {
  std::map<std::string, std::map<std::string, double>> namespaces;
  // 失败注入：failKey 的第 failOccurrence 次写入（1 起）返回 0 且不落盘。
  std::string failKey;
  int failOccurrence = 0;
  // 自上次配置失败注入以来，每个键被尝试写入的次数。
  std::map<std::string, int> putAttempts;
};

inline Store& store() {
  static Store instance;
  return instance;
}

// 测试钩子：返回 true 表示“当前电机仍在被驱动”。每次 put 若钩子为真，
// 就累加 writesWhileDriven，用于断言 NVS 写入没有发生在车轮驱动期间。
inline bool (*drivenHook)() = nullptr;
inline int writesWhileDriven = 0;

}  // namespace fake_preferences

class Preferences {
 public:
  // 打开命名空间。readOnly 仅记录，不限制假件写入。
  bool begin(const char* name, bool readOnly = false) {
    (void)readOnly;
    if (name == nullptr) return false;
    current_ = name;
    active_ = true;
    return true;
  }

  void end() {
    active_ = false;
    current_.clear();
  }

  // 写入接口返回真实字节数：成功为 1/4，未打开命名空间或被注入失败时为 0。
  size_t putUChar(const char* key, uint8_t value) { return put(key, value, 1); }
  size_t putChar(const char* key, int8_t value) { return put(key, value, 1); }
  size_t putFloat(const char* key, float value) { return put(key, value, 4); }
  size_t putUInt(const char* key, unsigned int value) {
    return put(key, value, 4);
  }

  uint8_t getUChar(const char* key, uint8_t defaultValue = 0) const {
    return static_cast<uint8_t>(get(key, defaultValue));
  }
  int8_t getChar(const char* key, int8_t defaultValue = 0) const {
    return static_cast<int8_t>(get(key, defaultValue));
  }
  float getFloat(const char* key,
                 float defaultValue = std::numeric_limits<float>::quiet_NaN())
      const {
    return static_cast<float>(get(key, defaultValue));
  }
  unsigned int getUInt(const char* key, unsigned int defaultValue = 0) const {
    return static_cast<unsigned int>(get(key, defaultValue));
  }

  // 清空当前命名空间。
  bool clear() {
    if (!active_) return false;
    fake_preferences::store().namespaces[current_].clear();
    return true;
  }

 private:
  size_t put(const char* key, double value, size_t bytes) {
    if (!active_ || key == nullptr) return 0;
    if (fake_preferences::drivenHook != nullptr && fake_preferences::drivenHook()) {
      ++fake_preferences::writesWhileDriven;
    }
    fake_preferences::Store& store = fake_preferences::store();
    const int attempt = ++store.putAttempts[key];
    if (!store.failKey.empty() && store.failKey == key &&
        attempt == store.failOccurrence) {
      return 0;  // 注入：本次写入失败且不落盘。
    }
    store.namespaces[current_][key] = value;
    return bytes;
  }

  double get(const char* key, double defaultValue) const {
    if (!active_ || key == nullptr) return defaultValue;
    std::map<std::string, std::map<std::string, double>>& namespaces =
        fake_preferences::store().namespaces;
    std::map<std::string, std::map<std::string, double>>::const_iterator ns =
        namespaces.find(current_);
    if (ns == namespaces.end()) return defaultValue;
    std::map<std::string, double>::const_iterator it = ns->second.find(key);
    if (it == ns->second.end()) return defaultValue;
    return it->second;
  }

  std::string current_;
  bool active_ = false;
};

// 清空所有命名空间与失败注入状态，测试用例开始时调用。
inline void fakePreferencesReset() {
  fake_preferences::store().namespaces.clear();
  fake_preferences::store().failKey.clear();
  fake_preferences::store().failOccurrence = 0;
  fake_preferences::store().putAttempts.clear();
  fake_preferences::writesWhileDriven = 0;
}

// 绕过 begin() 直接写入，供测试构造 NVS 状态（如版本不匹配）。
inline void fakePreferencesPutFloat(const char* ns, const char* key, float value) {
  fake_preferences::store().namespaces[ns][key] = value;
}

// 注入写入失败：failKey 的第 occurrence 次写入（1 起）返回 0 且不落盘。
// 调用时清零按键计数，便于按“第几次写”精确注入。occurrence<=0 表示不注入。
inline void fakePreferencesFailOnKey(const char* key, int occurrence = 1) {
  fake_preferences::Store& store = fake_preferences::store();
  store.putAttempts.clear();
  store.failKey = (key != nullptr) ? key : "";
  store.failOccurrence = occurrence;
}

// 关闭失败注入并清零按键计数。
inline void fakePreferencesClearFailures() {
  fake_preferences::store().failKey.clear();
  fake_preferences::store().failOccurrence = 0;
  fake_preferences::store().putAttempts.clear();
}

// 命名空间内是否存在指定键（用于直接断言键名拼写）。
inline bool fakePreferencesHasKey(const char* ns, const char* key) {
  const std::map<std::string, std::map<std::string, double>>& namespaces =
      fake_preferences::store().namespaces;
  std::map<std::string, std::map<std::string, double>>::const_iterator it =
      namespaces.find(ns);
  if (it == namespaces.end()) return false;
  return it->second.find(key) != it->second.end();
}
