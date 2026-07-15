#ifndef MHI_AC_CTRL_FIRMWARE_IDENTITY_H
#define MHI_AC_CTRL_FIRMWARE_IDENTITY_H

#include <string>

#ifdef USE_ESP_IDF
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#endif

inline std::string firmware_elf_sha256() {
#ifdef USE_ESP_IDF
  const char* sha = esp_app_get_elf_sha256_str();
  if (sha == nullptr) {
    return std::string("unknown");
  }
  return std::string(sha);
#else
  return std::string("not-esp-idf");
#endif
}

inline std::string firmware_app_description() {
#ifdef USE_ESP_IDF
  const esp_app_desc_t* desc = esp_app_get_description();
  if (desc == nullptr) {
    return std::string("unknown");
  }

  return std::string(desc->project_name) + " version=" + desc->version + " built=" + desc->date + " " + desc->time +
         " idf=" + desc->idf_ver;
#else
  return std::string("not-esp-idf");
#endif
}

inline std::string firmware_running_partition() {
#ifdef USE_ESP_IDF
  const esp_partition_t* part = esp_ota_get_running_partition();
  if (part == nullptr) {
    return std::string("unknown");
  }

  return std::string(part->label);
#else
  return std::string("not-esp-idf");
#endif
}

#endif  // MHI_AC_CTRL_FIRMWARE_IDENTITY_H
