#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
namespace marauder {
struct StorageReader { void *handle{}; };
struct DirEntry { char name[64]; uint32_t size; uint8_t is_dir; uint8_t pad[3]; };
esp_err_t storage_init();
bool storage_retry_mount();
esp_err_t storage_last_error();
const char *storage_last_stage();
int storage_last_errno();
bool storage_capture_begin(uint32_t frequency_mhz);
void storage_capture_end();
bool storage_append_iq8(uint32_t frequency_mhz, const int8_t *iq, size_t count);
void storage_log_position(uint32_t frequency_mhz, const char *utc,
                          double latitude, double longitude);
bool storage_ready();
bool storage_accessible();
bool storage_file_exists(const char *relative_path);
size_t storage_list_files(const char *relative_dir, const char *extension,
                          char names[][64], size_t capacity);
size_t storage_list_media_files(char names[][64], size_t capacity);
// Folder browsing. Entries are ordered folders-first, then case-insensitive by name. Pass anchor=nullptr for the first
// page (before=false) or the last page (before=true); otherwise the page that follows / precedes the anchor entry.
// Only `capacity` entries are held in memory, so any directory size works. *total = number of entries in the folder.
size_t storage_dir_page(const char *relative_dir, const DirEntry *anchor, bool before, DirEntry *out, size_t capacity, size_t *total);
bool storage_make_dir(const char *relative_dir);
// Plain file handles for streaming (map cache). mode as fopen: "rb", "wb", "ab".
void *storage_file_open(const char *relative_path, const char *mode);
size_t storage_file_read(void *handle, void *buffer, size_t bytes);
bool storage_file_write(void *handle, const void *buffer, size_t bytes);
bool storage_file_seek(void *handle, size_t offset);
void storage_file_close(void *handle);
size_t storage_read_file(const char *relative_path, size_t offset, void *buffer, size_t bytes);
bool storage_write_file(const char *relative_path, const void *buffer, size_t bytes, bool append);
bool storage_delete_file(const char *relative_path);
size_t storage_file_size(const char *relative_path);
bool storage_reader_open(StorageReader *reader, const char *relative_path);
size_t storage_reader_read(StorageReader *reader, void *buffer, size_t bytes);
bool storage_reader_skip(StorageReader *reader, size_t bytes);
void storage_reader_close(StorageReader *reader);
bool storage_gps_track_start(const char *name);
bool storage_gps_track_append(const struct GpsFix &fix);
void storage_gps_track_pause(bool paused);
void storage_gps_track_stop();
bool storage_gps_track_active();
bool storage_gps_track_paused();
uint32_t storage_gps_track_points();
uint32_t storage_gps_track_elapsed_seconds();
bool storage_gps_save_location(const char *name,const struct GpsFix &fix);
bool storage_gps_delete_location(uint32_t index);
}
