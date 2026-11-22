// Minimal esp_http_server declarations used by companion.ino -- syntax check only.
#pragma once
#include <cstddef>
#include <cstdint>
#include "esp_camera.h"
typedef void* httpd_handle_t;
typedef enum { HTTP_GET } httpd_method_t;
struct httpd_req_t {};
typedef struct { uint16_t server_port; uint16_t ctrl_port; } httpd_config_t;
#define HTTPD_DEFAULT_CONFIG() httpd_config_t{80, 32768}
typedef struct { const char* uri; httpd_method_t method; esp_err_t (*handler)(httpd_req_t*); void* user_ctx; } httpd_uri_t;
esp_err_t httpd_start(httpd_handle_t*, const httpd_config_t*);
esp_err_t httpd_register_uri_handler(httpd_handle_t, const httpd_uri_t*);
esp_err_t httpd_resp_set_type(httpd_req_t*, const char*);
esp_err_t httpd_resp_set_hdr(httpd_req_t*, const char*, const char*);
esp_err_t httpd_resp_send_chunk(httpd_req_t*, const char*, long);
