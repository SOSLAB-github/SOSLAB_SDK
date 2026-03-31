#ifndef SOSLAB_RING_BUFFER_H_
#define SOSLAB_RING_BUFFER_H_

#include <vector>
#include <mutex>
#include <condition_variable>

namespace soslab {
    template <typename T>
    class RingBuffer {
    public:
        explicit RingBuffer(size_t capacity)
            : buffer(capacity), capacity(capacity), head(0), tail(0), isFull(false) {}

        // -----------------------
        // push (copy)
        // -----------------------
        bool push(const T& item) {
            std::unique_lock<std::mutex> lock(mutex);
            if (isFull) return false;

            buffer[head] = item;
            head = (head + 1) % capacity;
            isFull = (head == tail);
            lock.unlock();
            notEmptyCv.notify_one();
            return true;
        }

        // -----------------------
        // push (move)
        // -----------------------
        bool push(T&& item)
        {
            std::unique_lock<std::mutex> lock(mutex);
            if (isFull) return false;

            buffer[head] = std::move(item);
            head = (head + 1) % capacity;
            isFull = (head == tail);

            lock.unlock();
            notEmptyCv.notify_one();
            return true;
        }

        bool pop(T& out, float timeout_ms = 1000.0f)
        {
            std::unique_lock<std::mutex> lock(mutex);

            bool woke = notEmptyCv.wait_for(
                lock,
                std::chrono::milliseconds(static_cast<int>(timeout_ms)),
                [this] { return !empty(); }
            );

            if (!woke) return false;
            if (empty()) return false;

            out = std::move(buffer[tail]);
            buffer[tail] = T{};
            tail = (tail + 1) % capacity;
            isFull = false;

            return true;
        }

        bool try_pop(T& out)
        {
            std::unique_lock<std::mutex> lock(mutex);
            if (empty()) return false;

            out = std::move(buffer[tail]);
            buffer[tail] = T{};
            tail = (tail + 1) % capacity;
            isFull = false;
            return true;
        }

        bool empty() const {
            return (!isFull && (head == tail));
        }

        bool full() const {
            return isFull;
        }

        size_t size() const {
            if (isFull) return capacity;
            if (head >= tail) return head - tail;
            return capacity + head - tail;
        }

        size_t getCapacity() const {
            return capacity;
        }

        void clear() {
            std::unique_lock<std::mutex> lock(mutex);
            head = tail;
            isFull = false;
        }

        void close() {
            std::unique_lock<std::mutex> lock(mutex);
            notEmptyCv.notify_all();
        }

    private:
        std::vector<T> buffer;
        const size_t capacity;
        size_t head;
        size_t tail;
        bool isFull;

        mutable std::mutex mutex;
        std::condition_variable notEmptyCv;
    };
}
#endif // SOSLAB_RING_BUFFER_H_