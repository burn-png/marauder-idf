#include "include/storage.hpp"
#include "include/pins.hpp"
#include "include/spi_gate.hpp"
#include "include/gps.hpp"
#include "include/dir_window.hpp"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstring>
#include <strings.h>
#include <sys/stat.h>
#include <dirent.h>
#include <cctype>
#include <unistd.h>
#include <cmath>

namespace {
constexpr char kTag[] = "storage";
sdmmc_card_t *s_card;
FILE *s_capture;
unsigned s_capture_number;
bool s_ready;
esp_err_t s_mount_error{ESP_ERR_INVALID_STATE};
const char *s_last_stage{"NOT ATTEMPTED"};
int s_last_errno{};
char s_capture_path[32];
char s_track_path[64]; uint32_t s_track_points; bool s_track_active,s_track_paused;int64_t s_track_start_us,s_track_pause_us,s_track_paused_us,s_track_end_us;
uint32_t s_capture_frequency;
uint32_t s_capture_samples;
int64_t s_capture_start_us;
marauder::GpsFix s_capture_start_fix{};

bool make_path(const char *relative, char out[96]) {
    if (!relative || !relative[0] || strlen(relative) > 78 || relative[0] == '/' || strstr(relative, "..")) return false;
    if (strcmp(relative, ".") == 0) return snprintf(out, 96, "/sdcard") < 96;
    for (const unsigned char *p = (const unsigned char *)relative; *p; ++p)
        // Any printable character FAT allows in a name is accepted (photos are often called "IMG+1 [2].jpg");
        // path traversal is still blocked by the ".." and leading-"/" checks above. Bytes >= 0x80 pass through (UTF-8 names).
        if (*p < 0x20 || *p == 0x7F || strchr("\\:*?\"<>|", *p)) return false;
    return snprintf(out, 96, "/sdcard/%s", relative) < 96;
}

bool is_media_extension(const char *name) {
    const char *dot = strrchr(name, '.');
    if (!dot) return false;
    return strcasecmp(dot, ".jpg") == 0 || strcasecmp(dot, ".jpeg") == 0 ||
           strcasecmp(dot, ".png") == 0 || strcasecmp(dot, ".mp4") == 0;
}

void list_media_recursive(const char *directory, unsigned depth, char names[][64],
                          size_t capacity, size_t &count, size_t &visited,
                          size_t &directory_errors) {
    if (depth > 4 || count >= capacity || visited >= 128) return;
    char path[96];
    if (!make_path(directory, path)) return;
    DIR *dir = opendir(path);
    if (!dir) {
        ++directory_errors;
        ESP_LOGW(kTag, "media scan cannot open %s: errno=%d", path, errno);
        return;
    }
    ++visited;
    while (count < capacity && visited < 128) {
        errno = 0;
        struct dirent *entry = readdir(dir);
        if (!entry) {
            if (errno) {
                ++directory_errors;
                ESP_LOGW(kTag, "media scan cannot read %s: errno=%d", path, errno);
            }
            break;
        }
        if (entry->d_name[0] == '.') continue;
        const size_t dir_len = strlen(directory);
        const size_t name_len = strlen(entry->d_name);
        const bool root = strcmp(directory, ".") == 0;
        const size_t relative_len = root ? name_len : dir_len + 1 + name_len;
        if (relative_len > 63) continue;

        char relative[64];
        if (root) {
            memcpy(relative, entry->d_name, name_len + 1);
        } else {
            memcpy(relative, directory, dir_len);
            relative[dir_len] = '/';
            memcpy(relative + dir_len + 1, entry->d_name, name_len + 1);
        }

        bool is_directory = entry->d_type == DT_DIR;
        bool is_regular_file = entry->d_type == DT_REG;
        if (entry->d_type == DT_UNKNOWN) {
            char full_path[96];
            struct stat st{};
            if (make_path(relative, full_path) && stat(full_path, &st) == 0) {
                is_directory = S_ISDIR(st.st_mode);
                is_regular_file = S_ISREG(st.st_mode);
            }
        }
        if (is_directory) {
            list_media_recursive(relative, depth + 1, names, capacity, count, visited,
                                 directory_errors);
        } else if (is_regular_file && is_media_extension(entry->d_name)) {
            memcpy(names[count++], relative, relative_len + 1);
        }
    }
    closedir(dir);
}
}

namespace marauder {
esp_err_t storage_init() {
    spi_gate_lock();
    if (s_ready) {
        spi_gate_unlock();
        return ESP_OK;
    }
    // The display initializes SPI2_HOST on the shared Marauder SPI pins first.
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 4000;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = pins::sd_cs;
    slot.host_id = SPI2_HOST;
    esp_vfs_fat_mount_config_t mount{};
    mount.format_if_mount_failed = false;
    mount.max_files = 8;
    mount.allocation_unit_size = 16 * 1024;
    if (s_card) {
        const esp_err_t unmount_error = esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
        if (unmount_error != ESP_OK) {
            s_mount_error = unmount_error;
            ESP_LOGE(kTag, "could not release previous SD mount: %s",
                     esp_err_to_name(unmount_error));
            spi_gate_unlock();
            return unmount_error;
        }
        s_card = nullptr;
    }
    s_card = nullptr;
    s_last_stage = "CARD MOUNT";
    s_last_errno = 0;
    esp_err_t e = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot, &mount, &s_card);
    s_mount_error = e;
    s_ready = (e == ESP_OK);
    if (s_ready) {
        ESP_LOGI(kTag, "SD card mounted at /sdcard (CS=%d, SPI2, 4 MHz)", (int)pins::sd_cs);
        sdmmc_card_print_info(stdout, s_card);
        if (mkdir("/sdcard/APPS", 0775) != 0 && errno != EEXIST) {
            s_last_stage = "APPS MKDIR";
            s_last_errno = errno;
            ESP_LOGW(kTag, "could not create APPS directory: errno=%d", errno);
        }
        struct stat app_stat{};
        if (stat("/sdcard/APPS/ANALOG_TV.APP", &app_stat) != 0) {
            static constexpr char manifest[] =
                "MARAUDER_APP=1\nAPI=1\nNAME=Analog TV\nCATEGORY=Video\n"
                "VERSION=0.1.0\n"
                "DESCRIPTION=Metadata demo only; requires external analog video hardware.\n"
                "EXECUTION=DISABLED\n";
            FILE *app = fopen("/sdcard/APPS/ANALOG_TV.APP", "wb");
            if (!app) {
                ESP_LOGW(kTag, "could not install Analog TV metadata manifest");
            } else {
                const bool wrote = fwrite(manifest, 1, sizeof(manifest) - 1, app) == sizeof(manifest) - 1;
                const int close_result = fclose(app);
                if (!wrote || close_result != 0) {
                    ESP_LOGW(kTag, "could not install Analog TV metadata manifest");
                }
            }
        }
        struct stat st{};
        if (stat("/sdcard/WARDRIVE.CSV", &st) != 0) {
            FILE *f = fopen("/sdcard/WARDRIVE.CSV", "wb");
            if (f) { fputs("utc,frequency_mhz,latitude_deg,longitude_deg\r\n", f); fclose(f); }
        }
        DIR *root = opendir("/sdcard");
        size_t entries = 0;
        int read_error = 0;
        const int open_error = root ? 0 : errno;
        if (root) {
            for (;;) {
                errno = 0;
                if (!readdir(root)) {
                    read_error = errno;
                    break;
                }
                ++entries;
            }
            closedir(root);
        }
        if (!root || read_error) {
            const int root_error = root ? read_error : open_error;
            s_last_stage = root ? "ROOT READ" : "ROOT OPEN";
            s_last_errno = root_error;
            s_mount_error = ESP_FAIL;
            s_ready = false;
            ESP_LOGE(kTag, "SD mounted but /sdcard cannot be read: errno=%d", root_error);
            const esp_err_t unmount_error = esp_vfs_fat_sdcard_unmount("/sdcard", s_card);
            if (unmount_error != ESP_OK) {
                ESP_LOGW(kTag, "failed to release unreadable SD mount: %s",
                         esp_err_to_name(unmount_error));
            } else {
                s_card = nullptr;
            }
            e = ESP_FAIL;
        } else {
            s_last_stage = "READY";
            s_last_errno = 0;
            ESP_LOGI(kTag, "SD filesystem root is readable (%u directory entries)",
                     (unsigned)entries);
        }
    } else {
        s_last_stage = "CARD MOUNT";
        s_last_errno = errno;
        ESP_LOGW(kTag, "SD mount failed: %s", esp_err_to_name(e));
    }
    if (s_ready) s_mount_error = ESP_OK;
    spi_gate_unlock();
    return e;
}

bool storage_retry_mount() {
    static int64_t last_attempt_us;
    if (storage_accessible()) return true;
    const int64_t now = esp_timer_get_time();
    if (last_attempt_us && now - last_attempt_us < 5000000) return false;
    last_attempt_us = now;
    if (s_ready) {
        spi_gate_lock();
        bool accessible = s_ready && s_card && sdmmc_get_status(s_card) == ESP_OK;
        if (accessible) {
            DIR *root = opendir("/sdcard");
            accessible = root != nullptr;
            if (root) closedir(root);
        }
        if (s_ready && !accessible) {
            sdmmc_card_t *card = s_card;
            s_ready = false;
            s_mount_error = ESP_FAIL;
            if (card) {
                const esp_err_t unmount_error = esp_vfs_fat_sdcard_unmount("/sdcard", card);
                if (unmount_error != ESP_OK) {
                    ESP_LOGW(kTag, "failed to release inaccessible SD mount: %s",
                             esp_err_to_name(unmount_error));
                } else {
                    s_card = nullptr;
                }
            }
        }
        spi_gate_unlock();
    }
    ESP_LOGI(kTag, "retrying SD mount");
    return storage_init() == ESP_OK && storage_accessible();
}

esp_err_t storage_last_error() {
    spi_gate_lock();
    const esp_err_t error = s_mount_error;
    spi_gate_unlock();
    return error;
}

const char *storage_last_stage() {
    spi_gate_lock();
    const char *stage = s_last_stage;
    spi_gate_unlock();
    return stage;
}

int storage_last_errno() {
    spi_gate_lock();
    const int error = s_last_errno;
    spi_gate_unlock();
    return error;
}

bool storage_ready() { return s_ready; }
bool storage_accessible() {
    if (!s_ready || !s_card) return false;
    spi_gate_lock();
    bool accessible = sdmmc_get_status(s_card) == ESP_OK;
    if (accessible) {
        DIR *root = opendir("/sdcard");
        accessible = root != nullptr;
        if (root) closedir(root);
    }
    if (accessible) {
        s_mount_error = ESP_OK;
        s_last_stage = "READY";
        s_last_errno = 0;
    } else {
        if (s_mount_error == ESP_OK) s_mount_error = ESP_FAIL;
        s_last_stage = "CARD STATUS";
        s_last_errno = errno;
    }
    spi_gate_unlock();
    return accessible;
}

bool storage_file_exists(const char *relative_path) {
    if (!s_ready) return false;
    char path[96];
    if (!make_path(relative_path, path)) return false;
    struct stat st{};
    spi_gate_lock();
    const bool exists = stat(path, &st) == 0 && S_ISREG(st.st_mode);
    spi_gate_unlock();
    return exists;
}

bool storage_capture_begin(uint32_t frequency_mhz) {
    if (!s_ready) return false;
    char path[32];
    struct stat st{};
    bool unused = false;
    bool opened = false;
    spi_gate_lock();
    if (s_capture) {
        spi_gate_unlock();
        return false;
    }
    for (unsigned tries = 0; tries < 10000; ++tries) {
        snprintf(path, sizeof(path), "/sdcard/RX%04u.C8", s_capture_number++ % 10000);
        if (stat(path, &st) != 0) { unused = true; break; }
    }
    if (unused) {
        s_capture = fopen(path, "wb");
        if (s_capture) {
            strncpy(s_capture_path, path + 8, sizeof(s_capture_path) - 1);
            s_capture_path[sizeof(s_capture_path) - 1] = 0;
            s_capture_frequency = frequency_mhz;
            s_capture_samples = 0;
            s_capture_start_us = esp_timer_get_time();
            s_capture_start_fix = gps_current_fix();
            opened = true;
        }
    }
    spi_gate_unlock();
    if (!unused) return false;
    return opened;
}

void storage_capture_end() {
    spi_gate_lock();
    if (!s_capture) {
        spi_gate_unlock();
        return;
    }
    fflush(s_capture); fclose(s_capture); s_capture = nullptr;
    char metadata[48];
    snprintf(metadata, sizeof(metadata), "/sdcard/%s.META", s_capture_path);
    FILE *f = fopen(metadata, "wb");
    if (f) {
        const auto fix = gps_current_fix();
        const auto start_fix = s_capture_start_fix;
        const int64_t stop_us = esp_timer_get_time();
        char lat[20], lon[20];
        if (fix.valid) { snprintf(lat,sizeof(lat),"%.7f",fix.latitude); snprintf(lon,sizeof(lon),"%.7f",fix.longitude); }
        else { strcpy(lat,"NA"); strcpy(lon,"NA"); }
        char start_lat[20],start_lon[20];
        if(start_fix.valid){snprintf(start_lat,sizeof(start_lat),"%.7f",start_fix.latitude);snprintf(start_lon,sizeof(start_lon),"%.7f",start_fix.longitude);}else{strcpy(start_lat,"NA");strcpy(start_lon,"NA");}
        fprintf(f, "MARAUDER_CAPTURE=1\nNAME=%s\nUTC=%s\nSTART_UTC=%s\nSTART_UPTIME_MS=%llu\nDURATION_MS=%llu\nFREQUENCY_MHZ=%lu\nSAMPLE_RATE_HZ=16000000\nMODE=SPARSE_256_SAMPLE_SNAPSHOTS\nCONTINUOUS_STREAM=0\nFORMAT=IQ8_INTERLEAVED\nCOMPLEX_SAMPLES=%lu\nBYTES=%lu\nGPS_VALID=%u\nLATITUDE=%s\nLONGITUDE=%s\nGPS_ALTITUDE_M=%.2f\nGPS_FIX_QUALITY=%u\nGPS_SATELLITES=%u\nSTART_LATITUDE=%s\nSTART_LONGITUDE=%s\nSTART_ALTITUDE_M=%.2f\nSTART_FIX_QUALITY=%u\nGPS_POSITION_NOTE=RECEIVER_POSITION_NOT_SIGNAL_ORIGIN\n",
                s_capture_path,
                (fix.valid && fix.utc[0]) ? fix.utc : "UNAVAILABLE",
                (start_fix.valid&&start_fix.utc[0])?start_fix.utc:"UNAVAILABLE",
                (unsigned long long)(s_capture_start_us / 1000),
                (unsigned long long)((stop_us - s_capture_start_us) / 1000),
                (unsigned long)s_capture_frequency, (unsigned long)s_capture_samples,
                (unsigned long)(s_capture_samples * 2), fix.valid ? 1u : 0u, lat, lon,
                fix.has_altitude?fix.altitude_m:NAN,fix.fix_quality,fix.satellites,
                start_lat,start_lon,start_fix.has_altitude?start_fix.altitude_m:NAN,start_fix.fix_quality);
        fclose(f);
    }
    spi_gate_unlock();
}

bool storage_append_iq8(uint32_t, const int8_t *iq, size_t count) {
    if (!iq) return false;
    spi_gate_lock();
    bool ok = s_capture && fwrite(iq, 2, count, s_capture) == count && fflush(s_capture) == 0;
    if (ok) s_capture_samples += (uint32_t)count;
    spi_gate_unlock();
    return ok;
}

void storage_log_position(uint32_t frequency_mhz, const char *utc, double latitude, double longitude) {
    if (!s_ready) return;
    spi_gate_lock();
    FILE *f = s_capture ? fopen("/sdcard/WARDRIVE.CSV", "ab") : nullptr;
    if (f) {
        int64_t lat = (int64_t)std::llround(latitude * 10000000.0);
        int64_t lon = (int64_t)std::llround(longitude * 10000000.0);
        uint64_t alat = lat < 0 ? (uint64_t)-lat : (uint64_t)lat;
        uint64_t alon = lon < 0 ? (uint64_t)-lon : (uint64_t)lon;
        fprintf(f, "%s,%u,%s%llu.%07llu,%s%llu.%07llu\r\n",
                (utc && utc[0]) ? utc : "", (unsigned)frequency_mhz,
                lat < 0 ? "-" : "", (unsigned long long)(alat / 10000000),
                (unsigned long long)(alat % 10000000),
                lon < 0 ? "-" : "", (unsigned long long)(alon / 10000000),
                (unsigned long long)(alon % 10000000));
        fclose(f);
    }
    spi_gate_unlock();
}

size_t storage_list_files(const char *relative_dir, const char *extension, char names[][64], size_t capacity) {
    if (!s_ready || !names || !capacity) return 0;
    char path[96];
    if (!make_path(relative_dir, path)) return 0;
    spi_gate_lock();
    DIR *dir = opendir(path);
    size_t count = 0;
    if (dir) {
        while (count < capacity) {
            errno = 0;
            struct dirent *entry = readdir(dir);
            if (!entry) {
                if (errno) ESP_LOGW(kTag, "cannot read %s: errno=%d", path, errno);
                break;
            }
            if (entry->d_name[0] == '.') continue;
            if (extension) {
                size_t n = strlen(entry->d_name), x = strlen(extension);
                if (n < x || strcasecmp(entry->d_name + n - x, extension)) continue;
            }
            if (strlen(entry->d_name) >= sizeof(names[0])) continue;
            strncpy(names[count], entry->d_name, sizeof(names[count]) - 1);
            names[count][sizeof(names[count]) - 1] = 0;
            ++count;
        }
        closedir(dir);
    } else {
        ESP_LOGW(kTag, "cannot open %s: errno=%d", path, errno);
    }
    spi_gate_unlock();
    return count;
}

size_t storage_list_media_files(char names[][64], size_t capacity) {
    if (!s_ready || !names || !capacity) return 0;
    spi_gate_lock();
    size_t count = 0;
    size_t visited = 0;
    size_t directory_errors = 0;
    list_media_recursive(".", 0, names, capacity, count, visited, directory_errors);
    ESP_LOGI(kTag, "media scan complete: %u files, %u directories, %u read errors",
             (unsigned)count, (unsigned)visited, (unsigned)directory_errors);
    spi_gate_unlock();
    return count;
}

size_t storage_read_file(const char *relative_path, size_t offset, void *buffer, size_t bytes) {
    if (!s_ready || !buffer || !bytes || offset > 0x7fffffff) return 0;
    char path[96];
    if (!make_path(relative_path, path)) return 0;
    spi_gate_lock();
    FILE *f = fopen(path, "rb");
    size_t got = 0;
    if (f && fseek(f, (long)offset, SEEK_SET) == 0) got = fread(buffer, 1, bytes, f);
    if (f) fclose(f);
    spi_gate_unlock();
    return got;
}

bool storage_write_file(const char *relative_path, const void *buffer, size_t bytes, bool append) {
    if (!s_ready || (!buffer && bytes)) return false;
    char path[96];
    if (!make_path(relative_path, path)) return false;
    spi_gate_lock();
    FILE *f = fopen(path, append ? "ab" : "wb");
    bool ok = f && fwrite(buffer, 1, bytes, f) == bytes && fflush(f) == 0;
    if (f) fclose(f);
    spi_gate_unlock();
    return ok;
}

bool storage_delete_file(const char *relative_path) {
    if (!s_ready) return false;
    char path[96];
    if (!make_path(relative_path, path)) return false;
    spi_gate_lock(); bool ok = unlink(path) == 0; spi_gate_unlock(); return ok;
}

size_t storage_file_size(const char *relative_path) {
    if (!s_ready) return 0;
    char path[96]; struct stat st{};
    if (!make_path(relative_path, path)) return 0;
    spi_gate_lock(); int result = stat(path, &st); spi_gate_unlock();
    return result == 0 && st.st_size >= 0 ? (size_t)st.st_size : 0;
}

bool storage_reader_open(StorageReader *reader, const char *relative_path) {
    if (!reader || !s_ready) return false;
    reader->handle = nullptr;
    char path[96];
    if (!make_path(relative_path, path)) return false;
    spi_gate_lock();
    FILE *file = fopen(path, "rb");
    spi_gate_unlock();
    reader->handle = file;
    return file != nullptr;
}

size_t storage_reader_read(StorageReader *reader, void *buffer, size_t bytes) {
    if (!reader || !reader->handle || !buffer || !bytes) return 0;
    spi_gate_lock();
    const size_t count = fread(buffer, 1, bytes, static_cast<FILE *>(reader->handle));
    spi_gate_unlock();
    return count;
}

bool storage_reader_skip(StorageReader *reader, size_t bytes) {
    if (!reader || !reader->handle || bytes > static_cast<size_t>(LONG_MAX)) return false;
    spi_gate_lock();
    const bool ok = fseek(static_cast<FILE *>(reader->handle), static_cast<long>(bytes), SEEK_CUR) == 0;
    spi_gate_unlock();
    return ok;
}

void storage_reader_close(StorageReader *reader) {
    if (!reader || !reader->handle) return;
    spi_gate_lock();
    const int result = fclose(static_cast<FILE *>(reader->handle));
    spi_gate_unlock();
    reader->handle = nullptr;
    if (result != 0) ESP_LOGW(kTag, "media file close failed");
}

// ---- directory browsing (keyset paging, bounded RAM: a page of entries, never the whole directory) ----
size_t storage_dir_page(const char *relative_dir, const DirEntry *anchor, bool before, DirEntry *out, size_t capacity, size_t *total) {
    if (total) *total = 0;
    if (!s_ready || !out || !capacity) return 0;
    char path[96];
    if (!make_path(relative_dir, path)) return 0;
    DirEntry anchor_copy;                       // the caller may pass an entry that lives inside `out`
    if (anchor) { anchor_copy = *anchor; anchor = &anchor_copy; }
    spi_gate_lock();
    DIR *dir = opendir(path);
    size_t count = 0, all = 0;
    if (dir) {
        for (;;) {
            errno = 0;
            struct dirent *e = readdir(dir);
            if (!e) break;
            if (e->d_name[0] == '.' || strlen(e->d_name) >= sizeof(out[0].name)) continue;
            ++all;
            dir_window_offer(out, count, capacity, anchor, before, e->d_type == DT_DIR, e->d_name);
        }
        closedir(dir);
        for (size_t i = 0; i < count; ++i) {          // sizes only for the entries that are actually shown
            if (out[i].is_dir) continue;
            char full[160]; struct stat st{};
            if (snprintf(full, sizeof(full), "%s/%s", path, out[i].name) < (int)sizeof(full) && stat(full, &st) == 0) out[i].size = (uint32_t)st.st_size;
        }
    } else {
        ESP_LOGW(kTag, "cannot open %s: errno=%d", path, errno);
    }
    spi_gate_unlock();
    if (total) *total = all;
    return count;
}

bool storage_make_dir(const char *relative_dir) {
    if (!s_ready) return false;
    char path[96];
    if (!make_path(relative_dir, path)) return false;
    spi_gate_lock();
    const bool ok = mkdir(path, 0775) == 0 || errno == EEXIST;
    spi_gate_unlock();
    return ok;
}

// ---- general file handles (map indexer / cache) ----
void *storage_file_open(const char *relative_path, const char *mode) {
    if (!s_ready || !mode) return nullptr;
    char path[96];
    if (!make_path(relative_path, path)) return nullptr;
    spi_gate_lock();
    FILE *f = fopen(path, mode);
    spi_gate_unlock();
    return f;
}
size_t storage_file_read(void *h, void *buffer, size_t bytes) {
    if (!h || !buffer || !bytes) return 0;
    spi_gate_lock(); const size_t n = fread(buffer, 1, bytes, static_cast<FILE *>(h)); spi_gate_unlock(); return n;
}
bool storage_file_write(void *h, const void *buffer, size_t bytes) {
    if (!h || !buffer) return false;
    spi_gate_lock(); const bool ok = fwrite(buffer, 1, bytes, static_cast<FILE *>(h)) == bytes; spi_gate_unlock(); return ok;
}
bool storage_file_seek(void *h, size_t offset) {
    if (!h || offset > static_cast<size_t>(LONG_MAX)) return false;
    spi_gate_lock(); const bool ok = fseek(static_cast<FILE *>(h), static_cast<long>(offset), SEEK_SET) == 0; spi_gate_unlock(); return ok;
}
void storage_file_close(void *h) {
    if (!h) return;
    spi_gate_lock(); fclose(static_cast<FILE *>(h)); spi_gate_unlock();
}

bool storage_gps_track_start(const char *name) {
    if(!s_ready||!name||!name[0]||s_track_active)return false;
    char path[96]; if(snprintf(path,sizeof(path),"GPS/TRACKS/%s.CSV",name)>=sizeof(path))return false;
    spi_gate_lock();
    if (mkdir("/sdcard/GPS",0775) != 0 && errno != EEXIST) { spi_gate_unlock(); return false; }
    if (mkdir("/sdcard/GPS/TRACKS",0775) != 0 && errno != EEXIST) { spi_gate_unlock(); return false; }
    char full[96];struct stat st{};bool ok=make_path(path,full)&&stat(full,&st)!=0;FILE*f=ok?fopen(full,"wb"):nullptr;
    if(f){fputs("MARAUDER_GPS_TRACK=1\nFORMAT_VERSION=1\nUTC,LATITUDE_DEG,LONGITUDE_DEG,ALTITUDE_M,SPEED_MPS,COURSE_DEG,FIX_QUALITY,SATELLITES,HDOP\r\n",f);ok=fclose(f)==0;}
    if(ok){strncpy(s_track_path,path,sizeof(s_track_path)-1);s_track_path[sizeof(s_track_path)-1]=0;s_track_points=0;s_track_active=true;s_track_paused=false;s_track_start_us=esp_timer_get_time();s_track_pause_us=0;s_track_paused_us=0;s_track_end_us=0;}
    spi_gate_unlock();return ok;
}
bool storage_gps_track_append(const GpsFix &fix){
    if(!s_ready||!s_track_active||s_track_paused||!fix.valid)return false;
    char full[96];if(!make_path(s_track_path,full))return false;spi_gate_lock();FILE*f=fopen(full,"ab");bool ok=false;
    if(f){const int written=fprintf(f,"%s,%.7f,%.7f,%.2f,%.3f,%.2f,%u,%u,%.2f\r\n",fix.utc[0]?fix.utc:"",fix.latitude,fix.longitude,fix.has_altitude?fix.altitude_m:NAN,fix.has_speed?fix.speed_mps:NAN,fix.has_course?fix.course_deg:NAN,fix.fix_quality,fix.satellites,fix.has_hdop?fix.hdop:NAN);const int close_result=fclose(f);ok=written>0&&close_result==0;if(ok)++s_track_points;}
    spi_gate_unlock();return ok;
}
void storage_gps_track_pause(bool paused){if(!s_track_active||s_track_paused==paused)return;int64_t now=esp_timer_get_time();if(paused)s_track_pause_us=now;else if(s_track_pause_us){s_track_paused_us+=now-s_track_pause_us;s_track_pause_us=0;}s_track_paused=paused;}
void storage_gps_track_stop(){if(s_track_active){s_track_end_us=esp_timer_get_time();if(s_track_paused&&s_track_pause_us)s_track_paused_us+=s_track_end_us-s_track_pause_us;}s_track_active=false;s_track_paused=false;s_track_path[0]=0;}
bool storage_gps_track_active(){return s_track_active;}
bool storage_gps_track_paused(){return s_track_paused;}
uint32_t storage_gps_track_points(){return s_track_points;}
uint32_t storage_gps_track_elapsed_seconds(){int64_t end=s_track_active?esp_timer_get_time():(s_track_end_us?s_track_end_us:s_track_start_us);int64_t paused=s_track_paused_us;if(s_track_active&&s_track_paused&&s_track_pause_us)paused+=end-s_track_pause_us;int64_t elapsed=end-s_track_start_us-paused;return elapsed>0?(uint32_t)(elapsed/1000000):0;}
bool storage_gps_save_location(const char *name,const GpsFix &fix){
    if(!s_ready||!fix.valid||!name||!name[0])return false;
    char row[192];int n=snprintf(row,sizeof(row),"%s,%.7f,%.7f,%.2f,%s\r\n",name,fix.latitude,fix.longitude,fix.has_altitude?fix.altitude_m:NAN,fix.utc);
    if(n<=0||(size_t)n>=sizeof(row))return false;
    spi_gate_lock();
    if(mkdir("/sdcard/GPS",0775)!=0&&errno!=EEXIST){spi_gate_unlock();return false;}
    FILE *f=fopen("/sdcard/GPS/LOCATIONS.CSV","ab");
    const bool ok=f&&fwrite(row,1,(size_t)n,f)==(size_t)n&&fflush(f)==0;
    const int close_result=f?fclose(f):0;
    spi_gate_unlock();
    return ok&&close_result==0;
}
bool storage_gps_delete_location(uint32_t index){
    if(!s_ready)return false;
    spi_gate_lock();
    FILE*in=fopen("/sdcard/GPS/LOCATIONS.CSV","rb");if(!in){spi_gate_unlock();return false;}
    FILE*out=fopen("/sdcard/GPS/LOCATIONS.TMP","wb");if(!out){fclose(in);spi_gate_unlock();return false;}
    char row[256];uint32_t row_index=0;bool ok=true;while(fgets(row,sizeof(row),in)){if(row_index++!=index&&fputs(row,out)<0){ok=false;break;}}
    if(fclose(in)!=0)ok=false;
    if(fclose(out)!=0)ok=false;
    if(ok){unlink("/sdcard/GPS/LOCATIONS.BAK");if(rename("/sdcard/GPS/LOCATIONS.CSV","/sdcard/GPS/LOCATIONS.BAK")==0){if(rename("/sdcard/GPS/LOCATIONS.TMP","/sdcard/GPS/LOCATIONS.CSV")==0)unlink("/sdcard/GPS/LOCATIONS.BAK");else{rename("/sdcard/GPS/LOCATIONS.BAK","/sdcard/GPS/LOCATIONS.CSV");ok=false;}}else ok=false;}
    if(!ok)unlink("/sdcard/GPS/LOCATIONS.TMP");
    spi_gate_unlock();return ok;
}
}
