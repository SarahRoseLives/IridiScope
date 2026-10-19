// Minimal blocking HTTP(S) GET for non-Windows builds (libcurl).
// Windows keeps its native WinHTTP code in the callers.
#pragma once

#include <string>

namespace iridi {

// Performs a GET with an 8 s timeout, following redirects.
// Returns true when the transfer completed (whatever the HTTP status);
// `status` receives the HTTP status code, `body` the response body and,
// on failure, `error` (if non-null) a short description.
bool http_get_portable(const std::string &url, std::string &body, int &status,
                       std::string *error = nullptr);

// True when this build has a working HTTP client.
bool http_available();

} // namespace iridi
