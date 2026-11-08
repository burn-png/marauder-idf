#pragma once
#include <stddef.h>

namespace marauder {
enum class MediaKind { JPEG, PNG, MP4, OTHER };

size_t media_list_files(char names[][64], size_t capacity);
MediaKind media_file_kind(const char *name);
bool media_display_image(const char *name, char *message, size_t message_size);
}
