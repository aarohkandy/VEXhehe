#pragma once

namespace intake {
enum class Mode { kOff = 0, kCollect, kReverse };

void init(void);
void set_mode(Mode mode);
Mode mode(void);
void update(void);
}  // namespace intake
