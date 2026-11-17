/**
 * @file calibration_command.cpp
 * @brief Минимальный плоский JSON-парсер команд калибровки.
 */

#include "web/calibration_command.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace web {
namespace {

bool isSpace(char c) noexcept
{
    return c == ' ' || c == '\n' || c == '\r' || c == '\t';
}

void skipSpace(const char *body, std::size_t length, std::size_t &pos) noexcept
{
    while (pos < length && isSpace(body[pos])) {
        ++pos;
    }
}

bool readString(const char *body, std::size_t length, std::size_t &pos,
                char *out, std::size_t capacity) noexcept
{
    skipSpace(body, length, pos);
    if (pos >= length || body[pos] != '"') {
        return false;
    }
    ++pos;
    std::size_t used = 0;
    while (pos < length && body[pos] != '"') {
        if (body[pos] == '\\') {
            ++pos;  // экранирование не требуется для известных ключей
            if (pos >= length) {
                return false;
            }
        }
        if (used + 1 >= capacity) {
            return false;
        }
        out[used++] = body[pos++];
    }
    if (pos >= length) {
        return false;
    }
    ++pos;  // закрывающая кавычка
    out[used] = '\0';
    return true;
}

bool readToken(const char *body, std::size_t length, std::size_t &pos,
               char *out, std::size_t capacity) noexcept
{
    skipSpace(body, length, pos);
    std::size_t used = 0;
    while (pos < length && body[pos] != ',' && body[pos] != '}' && !isSpace(body[pos])) {
        if (used + 1 >= capacity) {
            return false;
        }
        out[used++] = body[pos++];
    }
    out[used] = '\0';
    return used > 0U;
}

bool parseU32(const char *text, std::uint32_t &out) noexcept
{
    char *end = nullptr;
    const unsigned long value = std::strtoul(text, &end, 10);
    if (end == text || *end != '\0') {
        return false;
    }
    out = static_cast<std::uint32_t>(value);
    return true;
}

bool parseU64(const char *text, std::uint64_t &out) noexcept
{
    char *end = nullptr;
    const unsigned long long value = std::strtoull(text, &end, 10);
    if (end == text || *end != '\0' || value == 0ULL) {
        return false;
    }
    out = static_cast<std::uint64_t>(value);
    return true;
}

bool parseAxis(const char *text, int &out) noexcept
{
    if (std::strcmp(text, "azimuth") == 0 || std::strcmp(text, "az") == 0 ||
        std::strcmp(text, "0") == 0) {
        out = 0;
        return true;
    }
    if (std::strcmp(text, "altitude") == 0 || std::strcmp(text, "alt") == 0 ||
        std::strcmp(text, "1") == 0) {
        out = 1;
        return true;
    }
    return false;
}

bool parseDirection(const char *text, int &out) noexcept
{
    if (std::strcmp(text, "forward") == 0 || std::strcmp(text, "cw") == 0 ||
        std::strcmp(text, "0") == 0) {
        out = 0;
        return true;
    }
    if (std::strcmp(text, "backward") == 0 || std::strcmp(text, "ccw") == 0 ||
        std::strcmp(text, "1") == 0) {
        out = 1;
        return true;
    }
    return false;
}

bool requiresSession(CalibrationCommandKind kind) noexcept
{
    return kind != CalibrationCommandKind::CreateSession &&
           kind != CalibrationCommandKind::Reset;
}

}  // namespace

CalibrationCommandResult parseCalibrationCommand(CalibrationCommandKind kind,
                                                 const char *body, std::size_t length,
                                                 CalibrationCommand &out) noexcept
{
    out = CalibrationCommand{};
    if (body == nullptr || length == 0) {
        return CalibrationCommandResult::InvalidJson;
    }

    bool seen_session = false;
    bool seen_token = false;
    bool seen_generation = false;
    bool seen_axis = false;
    bool seen_direction = false;
    bool seen_revolutions = false;
    bool seen_repeats = false;
    bool seen_rate = false;
    bool seen_index = false;

    std::size_t pos = 0;
    skipSpace(body, length, pos);
    if (pos >= length || body[pos] != '{') {
        return CalibrationCommandResult::InvalidJson;
    }
    ++pos;
    skipSpace(body, length, pos);
    if (pos < length && body[pos] == '}') {
        ++pos;
    } else {
        for (;;) {
            char key[48];
            if (!readString(body, length, pos, key, sizeof(key))) {
                return CalibrationCommandResult::InvalidJson;
            }
            skipSpace(body, length, pos);
            if (pos >= length || body[pos] != ':') {
                return CalibrationCommandResult::InvalidJson;
            }
            ++pos;
            skipSpace(body, length, pos);
            if (pos >= length) {
                return CalibrationCommandResult::InvalidJson;
            }

            char value[48];
            bool is_string = body[pos] == '"';
            if (is_string) {
                if (!readString(body, length, pos, value, sizeof(value))) {
                    return CalibrationCommandResult::InvalidJson;
                }
            } else if (!readToken(body, length, pos, value, sizeof(value))) {
                return CalibrationCommandResult::InvalidJson;
            }

            if (std::strcmp(key, "session_id") == 0) {
                if (!parseU32(value, out.session_id)) {
                    return CalibrationCommandResult::InvalidValue;
                }
                seen_session = true;
            } else if (std::strcmp(key, "token") == 0) {
                // Принимается и число, и строка: браузер передаёт строку,
                // чтобы не потерять точность 64-битного токена.
                if (!parseU64(value, out.token)) {
                    return CalibrationCommandResult::InvalidValue;
                }
                seen_token = true;
            } else if (std::strcmp(key, "index") == 0) {
                if (!parseU32(value, out.index)) {
                    return CalibrationCommandResult::InvalidValue;
                }
                seen_index = true;
            } else if (std::strcmp(key, "confirm") == 0) {
                out.confirm = std::strcmp(value, "true") == 0;
            } else if (std::strcmp(key, "motion_generation") == 0) {
                if (!parseU32(value, out.motion_generation)) {
                    return CalibrationCommandResult::InvalidValue;
                }
                seen_generation = true;
            } else if (std::strcmp(key, "revolutions") == 0) {
                std::uint32_t value_u32 = 0;
                if (!parseU32(value, value_u32) || value_u32 < 1U || value_u32 > 50U) {
                    return CalibrationCommandResult::InvalidValue;
                }
                out.revolutions = value_u32;
                seen_revolutions = true;
            } else if (std::strcmp(key, "repeats") == 0) {
                std::uint32_t value_u32 = 0;
                if (!parseU32(value, value_u32) || value_u32 < 1U || value_u32 > 8U) {
                    return CalibrationCommandResult::InvalidValue;
                }
                out.repeats = value_u32;
                seen_repeats = true;
            } else if (std::strcmp(key, "rate") == 0) {
                char *end = nullptr;
                const float value_f = std::strtof(value, &end);
                if (end == value || *end != '\0' || !std::isfinite(value_f) ||
                    std::fabs(value_f) > kMaxCalibrationRate) {
                    return CalibrationCommandResult::InvalidValue;
                }
                out.rate = value_f;
                seen_rate = true;
            } else if (std::strcmp(key, "axis") == 0) {
                if (!parseAxis(value, out.axis)) {
                    return CalibrationCommandResult::InvalidValue;
                }
                seen_axis = true;
            } else if (std::strcmp(key, "direction") == 0) {
                if (!parseDirection(value, out.direction)) {
                    return CalibrationCommandResult::InvalidValue;
                }
                seen_direction = true;
            } else if (std::strcmp(key, "acknowledge_low_repeatability") == 0) {
                out.acknowledge_low_repeatability = std::strcmp(value, "true") == 0;
            }
            skipSpace(body, length, pos);
            if (pos < length && body[pos] == ',') {
                ++pos;
                continue;
            }
            if (pos < length && body[pos] == '}') {
                ++pos;
                break;
            }
            return CalibrationCommandResult::InvalidJson;
        }
    }
    skipSpace(body, length, pos);
    if (pos != length) {
        return CalibrationCommandResult::InvalidJson;
    }

    switch (kind) {
        case CalibrationCommandKind::CreateSession:
            if (!seen_axis || !seen_direction || !seen_revolutions || !seen_repeats) {
                return CalibrationCommandResult::MissingField;
            }
            break;
        case CalibrationCommandKind::Move:
            if (!seen_session || !seen_token || !seen_generation || !seen_direction ||
                !seen_rate) {
                return CalibrationCommandResult::MissingField;
            }
            break;
        case CalibrationCommandKind::DeleteMeasurement:
            if (!seen_session || !seen_token || !seen_index) {
                return CalibrationCommandResult::MissingField;
            }
            break;
        case CalibrationCommandKind::Reset:
            if (!seen_axis) {
                return CalibrationCommandResult::MissingField;
            }
            if (!out.confirm) {
                // Необратимый сброс без явного подтверждения не принимается.
                return CalibrationCommandResult::InvalidValue;
            }
            break;
        default:
            if (requiresSession(kind) && (!seen_session || !seen_token)) {
                return CalibrationCommandResult::MissingField;
            }
            break;
    }
    if (seen_session && out.session_id == 0U) {
        return CalibrationCommandResult::InvalidValue;
    }
    if (seen_rate && out.rate == 0.0F) {
        return CalibrationCommandResult::InvalidValue;
    }
    return CalibrationCommandResult::Ok;
}

}  // namespace web
