#ifndef REHAB_EXO_UTILS_RINGBUFFER_HPP
#define REHAB_EXO_UTILS_RINGBUFFER_HPP

#include <vector>
#include <mutex>
#include <atomic>

namespace rehab_exo {
namespace utils {

template<typename T>
class RingBuffer {
public:
  explicit RingBuffer(size_t capacity) 
    : capacity_(capacity), head_(0), size_(0) {
    buffer_.resize(capacity_);
  }

  bool push(const T& item) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_[head_] = item;
    head_ = (head_ + 1) % capacity_;
    if (size_ < capacity_) size_++;
    return true;
  }

  bool pop(T& item) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (size_ == 0) return false;
    size_t tail = (head_ + capacity_ - size_) % capacity_;
    item = buffer_[tail];
    size_--;
    return true;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_ == 0;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = 0;
    size_ = 0;
  }

private:
  std::vector<T> buffer_;
  size_t capacity_;
  size_t head_;
  size_t size_;
  mutable std::mutex mutex_;
};

} // namespace utils
} // namespace rehab_exo

#endif // REHAB_EXO_UTILS_RINGBUFFER_HPP
