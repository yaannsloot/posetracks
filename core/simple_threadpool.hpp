#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include <queue>

#ifndef SIMPLE_THREADPOOL_HPP
#define SIMPLE_THREADPOOL_HPP

// https://stackoverflow.com/a/32593825
class SimpleThreadPool {
public:
	void Start(const uint32_t numthreads = std::thread::hardware_concurrency());
	void QueueJob(const std::function<void()>& job);
	void Stop();
	bool busy();
private:
    void ThreadLoop();
    bool should_terminate = false;           // Tells threads to stop looking for jobs
    std::mutex queue_mutex;                  // Prevents data races to the job queue
    std::condition_variable mutex_condition; // Allows threads to wait on new jobs or termination 
    std::vector<std::thread> threads;
    std::queue<std::function<void()>> jobs;
};

#endif
