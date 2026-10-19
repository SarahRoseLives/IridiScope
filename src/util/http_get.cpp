#include "util/http_get.h"

#if !defined(_WIN32) && defined(HAS_LIBCURL)
#include <curl/curl.h>
#include <mutex>
#endif

namespace iridi {

#if !defined(_WIN32) && defined(HAS_LIBCURL)

namespace {

size_t write_cb(char *ptr, size_t size, size_t nmemb, void *userdata) {
    auto *out = static_cast<std::string *>(userdata);
    out->append(ptr, size * nmemb);
    return size * nmemb;
}

void global_init_once() {
    static std::once_flag once;
    std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

} // namespace

bool http_available() { return true; }

bool http_get_portable(const std::string &url, std::string &body, int &status,
                       std::string *error) {
    global_init_once();
    status = 0;
    body.clear();

    CURL *c = curl_easy_init();
    if (!c) {
        if (error) *error = "curl init failed";
        return false;
    }
    curl_easy_setopt(c, CURLOPT_URL, url.c_str());
    curl_easy_setopt(c, CURLOPT_USERAGENT, "IridiScope");
    curl_easy_setopt(c, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(c, CURLOPT_MAXREDIRS, 5L);
    curl_easy_setopt(c, CURLOPT_CONNECTTIMEOUT, 8L);
    curl_easy_setopt(c, CURLOPT_TIMEOUT, 16L);
    curl_easy_setopt(c, CURLOPT_NOSIGNAL, 1L); // safe in worker threads
    curl_easy_setopt(c, CURLOPT_ACCEPT_ENCODING, ""); // any supported encoding
    curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(c, CURLOPT_WRITEDATA, &body);

    CURLcode rc = curl_easy_perform(c);
    if (rc == CURLE_OK) {
        long code = 0;
        curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &code);
        status = (int)code;
    } else if (error) {
        *error = curl_easy_strerror(rc);
    }
    curl_easy_cleanup(c);
    return rc == CURLE_OK;
}

#else

bool http_available() { return false; }

bool http_get_portable(const std::string &, std::string &body, int &status,
                       std::string *error) {
    body.clear();
    status = 0;
    if (error) *error = "HTTP support not built in (libcurl missing)";
    return false;
}

#endif

} // namespace iridi
