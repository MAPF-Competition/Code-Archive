#ifndef THREADPOOLSINGLETON_H
#define THREADPOOLSINGLETON_H

#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {
 private:
  std::vector<std::thread> workers;
  std::queue<std::function<void()>> tasks;
  std::mutex queueMutex;
  std::condition_variable condition;
  bool stop = false;

 public:
  ThreadPool(size_t threads) {
    for (size_t i = 0; i < threads; ++i) {
      workers.emplace_back([this] {
        while (true) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(this->queueMutex);
            this->condition.wait(
                lock, [this] { return this->stop || !this->tasks.empty(); });
            if (this->stop && this->tasks.empty()) {
              return;
            }
            task = std::move(this->tasks.front());
            this->tasks.pop();
          }
          task();
        }
      });
    }
  }

  template <class F, class... Args>
  auto enqueue(F&& f, Args&&... args) -> std::future<decltype(f(args...))> {
    using ReturnType = decltype(f(args...));
    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    std::future<ReturnType> res = task->get_future();
    {
      std::unique_lock<std::mutex> lock(queueMutex);
      if (stop) {
        throw std::runtime_error("enqueue on stopped ThreadPool");
      }
      tasks.emplace([task]() { (*task)(); });
    }
    condition.notify_one();
    return res;
  }

  ~ThreadPool() {
    {
      std::unique_lock<std::mutex> lock(queueMutex);
      stop = true;
    }
    condition.notify_all();
    for (std::thread& worker : workers) {
      worker.join();
    }
  }
};

class ThreadPoolSingleton {
 public:
  // Delete copy and move constructors and assign operators
  ThreadPoolSingleton(ThreadPoolSingleton const&) = delete;
  ThreadPoolSingleton(ThreadPoolSingleton&&) = delete;
  ThreadPoolSingleton& operator=(ThreadPoolSingleton const&) = delete;
  ThreadPoolSingleton& operator=(ThreadPoolSingleton&&) = delete;

  static ThreadPool& getInstance() {
    static ThreadPool instance(
        10);  // Creates a ThreadPool with 4 threads. Adjust as needed.
    return instance;
  }

 private:
  ThreadPoolSingleton() {}  // Private constructor to prevent instantiation.
};

#endif  // THREADPOOLSINGLETON_H