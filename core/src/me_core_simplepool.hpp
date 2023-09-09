/*
me_core_simplepool.hpp
Thread pool implementation for task based multithreading

Copyright (C) 2023 Ian Sloat

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef ME_CORE_SIMPLEPOOL_HPP
#define ME_CORE_SIMPLEPOOL_HPP

#include <me_core.hpp>
#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>
#include <future>

namespace me {

    namespace core {

        // https://stackoverflow.com/a/32593825
        // Modified to use std::future
        class SimplePool {
        public:
            void Start(const uint32_t numthreads = std::thread::hardware_concurrency());
            template<typename F, typename... Args>
            inline auto QueueJob(F&& f, Args&&... args) -> std::future<typename std::invoke_result<F, Args...>::type>;
            void Stop();
            bool Busy();
            bool Running();
            ~SimplePool();
        private:
            void ThreadLoop();
            bool should_terminate = false;           // Tells threads to stop looking for jobs
            std::mutex queue_mutex;                  // Prevents data races to the job queue
            std::condition_variable mutex_condition; // Allows threads to wait on new jobs or termination 
            std::vector<std::thread> threads;
            std::queue<std::function<void()>> jobs;
        };

        template<typename F, typename... Args>
        inline auto SimplePool::QueueJob(F&& f, Args&&... args) -> std::future<typename std::invoke_result<F, Args...>::type>
        {
            using return_type = typename std::invoke_result<F, Args...>::type;

            auto task = std::make_shared<std::packaged_task<return_type()> >(
                std::bind(std::forward<F>(f), std::forward<Args>(args)...)
            );

            std::future<return_type> res = task->get_future();
            {
                std::unique_lock<std::mutex> lock(queue_mutex);

                // don't allow enqueueing after stopping the pool
                if (should_terminate)
                    throw std::runtime_error("enqueue on stopped ThreadPool");

                jobs.emplace([task]() { (*task)(); });
            }
            mutex_condition.notify_one();
            return res;
        }

        std::vector<std::pair<size_t, size_t>> calculateSegments(size_t total_size, size_t num_threads);

        static SimplePool global_pool;

    }

}

#endif
