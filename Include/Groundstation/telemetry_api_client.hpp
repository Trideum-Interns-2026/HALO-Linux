#ifndef TELEMETRY_API_CLIENT_HPP
#define TELEMETRY_API_CLIENT_HPP

#include <curl/curl.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

class TelemetryApiClient {
public:
    using ErrorHandler = std::function<void(const std::string&)>;

    TelemetryApiClient(std::string url, ErrorHandler error_handler)
        : url_(std::move(url)), error_handler_(std::move(error_handler)) {
        const CURLcode result = curl_global_init(CURL_GLOBAL_DEFAULT);
        if (result != CURLE_OK) {
            throw std::runtime_error(curl_easy_strerror(result));
        }
        worker_ = std::thread(&TelemetryApiClient::sendPending, this);
    }

    ~TelemetryApiClient() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_one();
        if (worker_.joinable()) {
            worker_.join();
        }
        curl_global_cleanup();
    }

    TelemetryApiClient(const TelemetryApiClient&) = delete;
    TelemetryApiClient& operator=(const TelemetryApiClient&) = delete;

    void submit(std::string json) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_json_ = std::move(json);
        }
        condition_.notify_one();
    }

private:
    static std::size_t appendResponse(
        char* data, std::size_t size, std::size_t count, void* context) {
        auto* response = static_cast<std::string*>(context);
        response->append(data, size * count);
        return size * count;
    }

    void reportError(const std::string& error) {
        const auto now = std::chrono::steady_clock::now();
        if (!has_reported_error_ ||
            now - last_error_report_ >= std::chrono::seconds(5)) {
            error_handler_(error);
            last_error_report_ = now;
            has_reported_error_ = true;
        }
    }

    void sendPending() {
        CURL* curl = curl_easy_init();
        if (curl == nullptr) {
            reportError("Could not initialize libcurl");
            return;
        }

        curl_slist* headers = curl_slist_append(nullptr, "Content-Type: application/json");
        if (headers == nullptr) {
            curl_easy_cleanup(curl);
            reportError("Could not allocate HTTP request headers");
            return;
        }

        curl_easy_setopt(curl, CURLOPT_URL, url_.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, 1000L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 2000L);

        while (true) {
            std::string json;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] { return stopping_ || !pending_json_.empty(); });
                if (stopping_) {
                    break;
                }
                json = std::move(pending_json_);
                pending_json_.clear();
            }

            std::string response;
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(json.size()));
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &TelemetryApiClient::appendResponse);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

            const CURLcode result = curl_easy_perform(curl);
            if (result != CURLE_OK) {
                reportError(curl_easy_strerror(result));
                continue;
            }

            long status_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
            if (status_code < 200 || status_code >= 300) {
                reportError("API returned HTTP " + std::to_string(status_code) +
                            (response.empty() ? "" : ": " + response));
            }
        }

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }

    const std::string url_;
    const ErrorHandler error_handler_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::string pending_json_;
    bool stopping_{false};
    bool has_reported_error_{false};
    std::thread worker_;
    std::chrono::steady_clock::time_point last_error_report_{};
};

#endif
