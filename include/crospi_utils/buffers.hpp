#pragma once
#include <mutex>
#include <algorithm>

namespace etasl {

/**
 * @brief Multithreaded ringbuffer that automatically enlarges when deadlines are not met.
 *
 */
class FIFOBuffer {
    size_t capacity;
    size_t start;
    size_t len;
    double* data;
    bool finish;
    std::mutex mtx;
    // std::condition_variable c_full;
    std::condition_variable not_empty;
    std::condition_variable not_full;

public:
    FIFOBuffer(size_t _capacity)
        : capacity(_capacity)
    {
        // std::fill_n(data, capacity, 0);
        start = 0;
        len = 0;
        finish = false;
        data = new double[capacity];
    }

    void print()
    {
        fmt::print("start:{}, len:{}, finish:{}, buf : ", start, len, finish);
        for (size_t i = 0; i < len; ++i) {
            fmt::print("{} ", data[(start + i) % capacity]);
        }
        fmt::print("\n");
    }

    /**
     *
     *
     */
    void signal_finish()
    {
        std::unique_lock lck(mtx);
        finish = true;
        not_empty.notify_one();
    }

    /**
     * @brief write but instead of blocking when the buffer is full, it enlarges the data-structure.
     * @param arg  pointer to array
     * @param sz   number of array elements to push
     * @return false if could not write.
     */
    void write_non_blocking(const double* arg, int sz)
    {
        // idx+sz  <= capacity ==>   write [idx, idx+sz[
        // idx+sz  > capacity  ==>  write [idx, capacity[  and [0, remainder [
        // [idx,capacity[  has capacity - idx elements.
        // [0, remainer [  has sz - (capacity-idx) elements.
        std::unique_lock lck(mtx);
        // next index to write:
        size_t idx = (start + len) % capacity;
        if (sz > capacity - len) {
            size_t newcapacity = capacity;
            do {
                newcapacity = newcapacity * 2;
            } while (sz > newcapacity - len);
            // failed, FIFO queue is full
            // we will skip this write in total.
            double* data2 = new double[newcapacity];
            if (start + len <= capacity) {
                std::copy_n(data + start, len, data2);
            } else {
                size_t len_1 = capacity - start;
                std::copy_n(data + start, len_1, data2);
                std::copy_n(data, len - len_1, data2 + len_1);
            }
            delete[] data;
            data = data2;
            start = 0;
            capacity = newcapacity;
            idx = start + len;
        }
        if (idx + sz <= capacity) {
            // can be written in one step
            std::copy_n(arg, sz, data + idx);
        } else {
            // needs to be written in two pieces:
            size_t len_1 = capacity - idx;
            std::copy_n(arg, len_1, data + idx);
            std::copy_n(arg + len_1, sz - len_1, data);
        }
        len = len + sz;
        not_empty.notify_one();
    }

    /**
     * @brief write but instead of blocking when the buffer is full, returns false.
     * It also fails completely: i.e. none of the elements are pushed.
     * @param arg  pointer to array
     * @param sz   number of array elements to push
     * @return false if could not write.
     */
    bool try_write_non_blocking(const double* arg, int sz)
    {
        // idx+sz  <= capacity ==>   write [idx, idx+sz[
        // idx+sz  > capacity  ==>  write [idx, capacity[  and [0, remainder [
        // [idx,capacity[  has capacity - idx elements.
        // [0, remainer [  has sz - (capacity-idx) elements.
        std::unique_lock lck(mtx);

        if (sz > capacity - len) {
            // failed, FIFO queue is full
            // we will skip this write in total.
            return false;
        }
        // next index to write:
        size_t idx = (start + len) % capacity;
        if (idx + sz <= capacity) {
            // can be written in one step
            std::copy_n(arg, sz, data + idx);
        } else {
            // needs to be written in two pieces:
            size_t len_1 = capacity - idx;
            std::copy_n(arg, len_1, data + idx);
            std::copy_n(arg + len_1, sz - len_1, data);
        }
        len = len + sz;
        not_empty.notify_one();
        return true;
    }

    void write_blocking(const double* arg, int sz)
    {
        std::unique_lock lck(mtx);
        not_full.wait(lck, [&] { return sz <= capacity - len; });
        // next index to write:
        size_t idx = (start + len) % capacity;
        if (idx + sz <= capacity) {
            // can be written in one step
            std::copy_n(arg, sz, data + idx);
        } else {
            // needs to be written in two pieces:
            size_t len_1 = capacity - idx;
            std::copy_n(arg, len_1, data + idx);
            std::copy_n(arg + len_1, sz - len_1, data);
        }
        len = len + sz;
        not_empty.notify_one();
    }

    bool read_blocking(double* arg, size_t& sz)
    {
        std::unique_lock lck(mtx);
        not_empty.wait(lck, [&] { return len >= sz || finish; });
        if (len < sz) {
            // sz can only be something different as the input when finish was true.
            sz = len;
        }
        if (len == 0) {
            sz = 0;
            return true;
        }
        // if start + sz <= capacity  ==> read [start, start+sz[
        // if start + sz > capacity   ==> read [ start, capacity[  and [0, remainder [
        // [start, capacity[  has capacity - start elements
        // remainder = sz - (capacity-start)
        if (start + sz <= capacity) {
            std::copy_n(data + start, sz, arg);
        } else {
            size_t len_1 = capacity - start;
            std::copy_n(data + start, len_1, arg);
            std::copy_n(data, sz - len_1, arg + len_1);
        }
        start = (start + sz) % capacity;
        len = len - sz;
        not_full.notify_one();
        return false;
    }

    ~FIFOBuffer()
    {
        delete[] data;
    }
};


/**
 * @brief A ring buffer where a producer generates values and a consumer consumes the values
 * in block.  This can be used e.g. for asynchronous filewriting.
 *
 * The class provides the typical `T& front()` and `T& back()` member functions to access the
 * front and the back of the buffer.  `push_front()` and `push_back()` pushes values to the front and
 * the back.
 *
 * Additionally there is a  non-blocking `check()` member function that can be called by the *producer*. It will
 * check whether there are enough elements (> `blocksize`) in the buffer, and removes that block from the
 * circular buffer, and signal the consumer that there is a block available.  The consumer gets a reference
 * to the data to process using a blocking call to `getDataBlock()`
 *
 * The following assumptions are made:
 *   - one producer and one consumer.  The producer is a higher priority activity that should not be
 *     blocked for more than a very small time.  The consumer is best effort. We are not going to
 *     wait if the consumer can't do his job in time.
 *   - The producer is cyclic,  the consumer only activates when needed.
 *   - there is only signaling from producer to consumer.  It is never checked whether the consumer
 *     had time to finish its job.
 *
 * @tparam T type you want to buffer.
 */
template <typename T>
class RingBuffer {
    std::size_t size;
    std::size_t blocksize;
    std::size_t start;
    std::size_t n;
    T* data;

    std::condition_variable c_ready;
    std::mutex m;
    std::size_t block_start;
    std::size_t block_n;
    bool ready;

public:
    /**
     * @brief construct a ring buffer
     * @param N number of elements in the ringbuffer
     * @param blocksize
     */
    RingBuffer(std::size_t N, std::size_t blocksize)
        : size(N)
    {
        data = new T[size];
        start = 0;
        n = 0;
        ready = false;
    }

    /**
     * @return return reference to element in the front.
     */
    T& front()
    {
        return &data[start + n];
    }

    /**
     * @brief
     * @return returns reference to element in the back
     */
    T& back()
    {
        return data[start];
    }

    /**
     * @brief true if you can't pop blocksize elements
     * @param blocksize
     * @return
     */
    bool empty(std::size_t blocksize = 1)
    {
        return n < blocksize;
    }

    /**
     * @brief true if you can't push blocksize elements
     * @param blocksize
     * @return
     */
    bool full(std::size_t blocksize = 1)
    {
        return n + blocksize > size;
    }

    /**
     * @brief push value to the front (const ref)
     * @param value
     */
    void push_front(const T& value)
    {

        data[(start + n) % size] = value;
        if (n < size) {
            n = n + 1;
        } else {
            // overwrite:
            start = (start + 1) % size;
        }
    }
    /**
     * @brief push value to the front (move)
     * @param value
     */
    void push_front(T&& value)
    {

        data[(start + n) % size] = std::move(value);
        if (n < size) {
            n = n + 1;
        } else {
            // overwrite:
            start = (start + 1) % size;
        }
    }
    void push_back(const T& value)
    {
        data[(start - 1) % size] = value;
        start = (start - 1) % size;
        if (n < size) {
            n = n + 1;
        }
    }

    void push_back(T&& value)
    {
        data[(start - 1) % size] = value;
        start = (start - 1) % size;
        if (n < size) {
            n = n + 1;
        }
    }

    /**
     * @brief remove element in the front.
     * You have to check whether the buffer is empty.
     */
    void pop_front()
    {
        n = (n - 1) % size;
    }

    /**
     * @brief remove the element in the back
     * you have to check whether the buffer is empty.
     */
    void pop_back()
    {
        n = (n - 1) % size;
        start = (start + 1) % size;
    }

    std::size_t getN()
    {
        return n;
    }

    void check()
    {
        block_start = start;
        block_n = n;
        start = (start + n) % size;
        n = 0;
    }

    ~RingBuffer()
    {
        delete[] data;
    }
};

} // namespace etasl