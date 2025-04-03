#include <thread>
#include <vector>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <future>
#include <stdexcept>
#include <memory>

class ThreadPool {
public:
    // 构造函数：创建指定数量的工作线程
    explicit ThreadPool(size_t numThreads) {
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queueMutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        if (stop && tasks.empty()) {
                            return;  // 如果线程池停止且任务队列为空，则退出线程
                        }
                        task = std::move(tasks.front());  // 获取任务
                        tasks.pop();  // 从队列中移除任务
                    }
                    task();  // 执行任务
                }
            });
        }
    }

    // 析构函数：确保所有线程安全退出
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;  // 设置停止标志
        }
        condition.notify_all();  // 通知所有线程
        for (std::thread& worker : workers) {
            worker.join();  // 等待所有线程完成
        }
    }

    // 禁止拷贝构造和赋值
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    // 提交任务到线程池
    template <typename F, typename... Args>
    auto enqueue(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type> {
        using return_type = typename std::result_of<F(Args...)>::type;

        // 创建一个包装任务的 std::packaged_task
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        // 获取任务的未来结果
        std::future<return_type> res = task->get_future();

        {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (stop) {
                throw std::runtime_error("enqueue on stopped ThreadPool");
            }
            tasks.emplace([task]() { (*task)(); });  // 将任务加入队列
        }

        condition.notify_one();  // 通知一个线程
        return res;
    }

private:
    std::vector<std::thread> workers;  // 存储线程池中的线程
    std::queue<std::function<void()>> tasks;  // 任务队列
    std::mutex queueMutex;  // 保护任务队列的互斥锁
    std::condition_variable condition;  // 条件变量，用于线程间同步
    bool stop = false;  // 线程池停止标志
};