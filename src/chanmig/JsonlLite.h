/* JsonlLite — minimal field extraction from devourer machine-event lines.
 *
 * Devourer events are FLAT JSON objects, one per line, first field always
 * "ev" (src/Event.h contract). Consumers inside the chanmig stack (the scout
 * tail-following the primary receiver's JSONL, the replay tools) need a
 * handful of typed lookups, not a JSON library: this is a single-pass scanner
 * that tracks string state (so a "text" value containing '"key":' can never
 * false-match), understands the exact value shapes Ev emits — numbers,
 * strings with its escape set, true/false/null, flat arrays of numbers — and
 * nothing else. Non-event lines (stderr noise, partial writes) simply fail
 * every lookup. Pure, header-only, selftested. */
#ifndef DEVOURER_CHANMIG_JSONL_LITE_H
#define DEVOURER_CHANMIG_JSONL_LITE_H

#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

namespace devourer {
namespace chanmig {

namespace jsonl_detail {
/* Locate the raw value span of "key": in a flat object. Returns false when
 * the key is absent (or only occurs inside a string value). */
inline bool find_value(std::string_view line, std::string_view key,
                       size_t &vbegin, size_t &vend) {
  size_t i = 0;
  const size_t n = line.size();
  while (i < n) {
    const char c = line[i];
    if (c == '"') {
      /* A string at top level is either a key (followed by ':') or a string
       * value. Scan it with escapes. */
      const size_t sbegin = i + 1;
      size_t j = sbegin;
      while (j < n && line[j] != '"') {
        if (line[j] == '\\')
          ++j;
        ++j;
      }
      if (j >= n)
        return false; /* unterminated — not a complete event line */
      const size_t send = j; /* exclusive */
      i = j + 1;
      /* key? */
      if (i < n && line[i] == ':') {
        ++i;
        if (line.substr(sbegin, send - sbegin) == key) {
          vbegin = i;
          /* value runs to the matching top-level ',' or '}'. Strings and
           * arrays are skipped structurally; nested objects don't occur in
           * this schema but are skipped defensively. */
          int depth = 0;
          size_t k = i;
          while (k < n) {
            const char v = line[k];
            if (v == '"') {
              ++k;
              while (k < n && line[k] != '"') {
                if (line[k] == '\\')
                  ++k;
                ++k;
              }
              if (k >= n)
                return false;
            } else if (v == '[' || v == '{') {
              ++depth;
            } else if (v == ']' || v == '}') {
              if (depth == 0)
                break;
              --depth;
            } else if (v == ',' && depth == 0) {
              break;
            }
            ++k;
          }
          vend = k;
          return vend > vbegin;
        }
        /* not our key: fall through, the value scanner below advances */
      }
      continue;
    }
    ++i;
  }
  return false;
}
} /* namespace jsonl_detail */

/* True when the line is an event line named `name` (first-field contract:
 * the line starts with {"ev":"<name>"). */
inline bool jsonl_ev_is(std::string_view line, std::string_view name) {
  static const char prefix[] = "{\"ev\":\"";
  if (line.size() < sizeof(prefix) - 1 + name.size() + 1)
    return false;
  if (line.compare(0, sizeof(prefix) - 1, prefix) != 0)
    return false;
  if (line.compare(sizeof(prefix) - 1, name.size(), name) != 0)
    return false;
  return line[sizeof(prefix) - 1 + name.size()] == '"';
}

/* Number lookup. false when absent, null, or not a number. */
inline bool jsonl_num(std::string_view line, std::string_view key,
                      double *out) {
  size_t b, e;
  if (!jsonl_detail::find_value(line, key, b, e))
    return false;
  const std::string v(line.substr(b, e - b));
  if (v == "null" || v.empty() || v[0] == '"' || v[0] == '[')
    return false;
  char *end = nullptr;
  const double d = std::strtod(v.c_str(), &end);
  if (end == v.c_str())
    return false;
  *out = d;
  return true;
}

inline bool jsonl_int(std::string_view line, std::string_view key,
                      long long *out) {
  double d;
  if (!jsonl_num(line, key, &d))
    return false;
  *out = static_cast<long long>(d);
  return true;
}

/* String lookup with the Ev escape set unescaped (\" \\ \n \t \r; \uXXXX
 * folds to '?' — the schema only emits it for control bytes). */
inline bool jsonl_str(std::string_view line, std::string_view key,
                      std::string *out) {
  size_t b, e;
  if (!jsonl_detail::find_value(line, key, b, e))
    return false;
  if (e - b < 2 || line[b] != '"' || line[e - 1] != '"')
    return false;
  out->clear();
  for (size_t i = b + 1; i < e - 1; ++i) {
    char c = line[i];
    if (c == '\\' && i + 1 < e - 1) {
      const char x = line[++i];
      switch (x) {
      case 'n': c = '\n'; break;
      case 't': c = '\t'; break;
      case 'r': c = '\r'; break;
      case 'u':
        i += 4 < e - 1 - i ? 4 : e - 2 - i;
        c = '?';
        break;
      default: c = x; break;
      }
    }
    out->push_back(c);
  }
  return true;
}

/* Flat numeric array lookup: fills out[0..max) and sets *n to the element
 * count found (clamped to max). false when absent or not an array. */
inline bool jsonl_arr(std::string_view line, std::string_view key, int *out,
                      int max, int *n) {
  size_t b, e;
  if (!jsonl_detail::find_value(line, key, b, e))
    return false;
  if (line[b] != '[')
    return false;
  int count = 0;
  size_t i = b + 1;
  while (i < e && count < max) {
    while (i < e && (line[i] == ',' || line[i] == ' '))
      ++i;
    if (i >= e || line[i] == ']')
      break;
    const std::string v(line.substr(i, e - i));
    char *end = nullptr;
    const long val = std::strtol(v.c_str(), &end, 10);
    if (end == v.c_str())
      break;
    out[count++] = static_cast<int>(val);
    i += static_cast<size_t>(end - v.c_str());
  }
  *n = count;
  return true;
}

} /* namespace chanmig */
} /* namespace devourer */

#endif /* DEVOURER_CHANMIG_JSONL_LITE_H */
