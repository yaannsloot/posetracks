/*
Copyright (C) 2024 Ian Sloat

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

#include "simplepool.hpp"
#include <memory>

namespace me::threading {

    void SimplePool::Start(const uint32_t numthreads) {
        threads.resize(numthreads);
        for (uint32_t i = 0; i < numthreads; i++) {
            threads.at(i) = std::thread([this] { this->ThreadLoop(); });
        }
    }

    void SimplePool::ThreadLoop() {
        while (true) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                mutex_condition.wait(lock, [this] {
                    return !jobs.empty() || should_terminate;
                    });
                if (should_terminate) {
                    return;
                }
                job = jobs.front();
                jobs.pop();
            }
            job();
        }
    }

    bool SimplePool::Busy() {
        bool poolbusy;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            poolbusy = !jobs.empty();
        }
        return poolbusy;
    }

    bool SimplePool::Running() {
        bool poolrunning;
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            poolrunning = !threads.empty();
        }
        return poolrunning;
    }

    void SimplePool::Stop() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            should_terminate = true;
        }
        mutex_condition.notify_all();
        for (std::thread& active_thread : threads) {
            active_thread.join();
        }
        threads.clear();
    }

    size_t SimplePool::NumThreads() {
        return threads.size();
    }

    SimplePool::~SimplePool() {
        if (Running())
            Stop();
    }

    std::vector<std::pair<size_t, size_t>> calculateSegments(size_t total_size, size_t num_threads) {
        std::vector<std::pair<size_t, size_t>> segments;

        // Guard
        if (total_size < 1 || num_threads < 1)
            return segments;

        // Calculate the size of each segment
        size_t segment_size = total_size / num_threads;
        if (total_size % num_threads != 0) {
            segment_size++;
        }

        // Calculate the segments
        size_t start = 0;
        while (start < total_size) {
            size_t end = start + segment_size;
            if (end > total_size) {
                end = total_size;
            }
            segments.push_back(std::make_pair(start, end));
            start = end;
        }

        return segments;
    }

}