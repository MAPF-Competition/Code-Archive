#pragma once
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>

namespace SchedulerUtils
{
    class APSPThreadPool
    {
    public:
        explicit APSPThreadPool(size_t num_threads)
            : stop(false)
        {
            for (size_t i = 0; i < num_threads; ++i)
            {
                workers.emplace_back([this]
                                     {
                    while (true)
                    {
                        std::function<void()> task;
                        {
                            std::unique_lock<std::mutex> lock(queue_mutex);
                            condition.wait(lock, [this] {
                                return stop || !tasks.empty();
                            });

                            if (stop && tasks.empty())
                                return;

                            task = std::move(tasks.front());
                            tasks.pop();
                        }
                        task();
                    } });
            }
        }

        template <class F, class... Args>
        auto enqueue(F &&f, Args &&...args)
            -> std::future<typename std::result_of<F(Args...)>::type>
        {
            using return_type = typename std::result_of<F(Args...)>::type;

            auto task = std::make_shared<std::packaged_task<return_type()>>(
                std::bind(std::forward<F>(f), std::forward<Args>(args)...));

            std::future<return_type> res = task->get_future();
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                if (stop)
                    throw std::runtime_error("タスクの追加が停止されたスレッドプールです");

                tasks.emplace([task]()
                              { (*task)(); });
            }
            condition.notify_one();
            return res;
        }

        ~APSPThreadPool()
        {
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                stop = true;
            }
            condition.notify_all();
            for (std::thread &worker : workers)
                worker.join();
        }

    private:
        std::vector<std::thread> workers;
        std::queue<std::function<void()>> tasks;

        std::mutex queue_mutex;
        std::condition_variable condition;
        bool stop;
    };
}