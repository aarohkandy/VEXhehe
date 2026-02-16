#pragma once

namespace indexer {
enum class Mode { kOff = 0, kFeedForward, kFeedReverse };

void init(void);
void set_mode(Mode mode);
Mode mode(void);
void update(void);
}  // namespace indexer
