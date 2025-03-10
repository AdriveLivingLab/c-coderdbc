#include "formatter.h"
#include <algorithm>
#include <sstream>
#include <limits>
#include <iomanip>

static const size_t kMaxWorkArrLength = 4096;

static char work_buff[kMaxWorkArrLength] = { 0 };

static const std::string __typeprint[8] =
{
  "int8_t",
  "int16_t",
  "int32_t",
  "int64_t",
  "uint8_t",
  "uint16_t",
  "uint32_t",
  "uint64_t"
};

//(added by AdriveLivingLab)
static const std::string __msgtypeprint[8] =
{
  "int8",
  "int16",
  "int32",
  "int64",
  "uint8",
  "uint16",
  "uint32",
  "uint64"
};

std::string IndentedString(size_t n, const std::string& source, const char c)
{
  if (source.length() >= n)
  {
    return source;
  }
  else
  {
    std::string indent(n - source.length(), c);
    return source + indent;
  }
}

const char* StrPrint(const char* format, ...)
{
  va_list args;
  va_start(args, format);

  vsnprintf(work_buff, kMaxWorkArrLength, format, args);

  va_end(args);
  return work_buff;
}

std::string PrintType(uint8_t id)
{
  if (id < 8)
  {
    return __typeprint[id];
  }

  return "";
}

//(added by AdriveLivingLab)
std::string PrintMsgType(uint8_t id)
{
  if (id < 8)
  {
    return __msgtypeprint[id];
  }

  return "";
}

std::string str_toupper(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c)
  {
    return std::toupper(c);
  });
  return s;
}

std::string str_tolower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c)
  {
    return std::tolower(c);
  });
  return s;
}

std::string str_trim(std::string s)
{
  size_t passed = 0;

  if (s.empty())
  {
    return s + '\n';
  }

  passed = 0;

  while (passed < s.size())
  {
    if (s[s.size() - passed - 1] > ' ')
    {
      break;
    }

    ++passed;
  }

  if (passed != 0)
  {
    // remove tail with non-printable values
    s.erase(s.size() - passed, passed);
  }

  return s;
}

template<char L, char U>
static inline bool in_range(const char c)
{
  return ((c >= L) && (c <= U));
}

static bool is_first_valid(const char c)
{
  return in_range<'a', 'z'>(c) || in_range<'A', 'Z'>(c) || c == '_';
}

static bool is_nonfirst_valid(const char c)
{
  return is_first_valid(c) || in_range<'0', '9'>(c);
}

std::string make_c_name(const std::string& s)
{
  std::string ret{};

  if (s.length() == 0)
  {
    return ret;
  }

  for (auto i = 0; i < s.length(); i++)
  {
    if ((ret.length() == 0 && is_first_valid(s[i])) ||
      (ret.length() > 0 && is_nonfirst_valid(s[i])))
    {
      // basic C-identifier rule
      ret += s[i];
    }
    else if (s[i] == ' ')
    {
      // special case for whitespaces
      ret += '_';
    }
  }

  return ret;
}

std::string prt_double(double value, size_t precision, bool usedot)
{
  std::stringstream strstrm;
  strstrm.imbue(std::locale::classic());
  strstrm << std::fixed << std::setprecision(10) << value;

  std::string s(strstrm.str());

  size_t dotpos = s.find('.');

  if (dotpos == std::string::npos)
  {
    // dot not found
    if (usedot)
    {
      s += ".0";
    }

    return s;
  }

  // remove trailing zeros after decimal delimiter (dot char)
  size_t tailsize = std::min((s.size() - (dotpos + 1u)), precision);
  size_t addtail = 0u;

  for (size_t j = 0; j < tailsize; j++)
  {
    auto ch =  s[dotpos + 1u + j];

    if (ch > '0' && ch <= '9')
    {
      addtail = j + 1;
    }
  }

  if (addtail == 0)
  {
    // precision == 0 or xxx.0(0)  resize cut tail with dot
    s.resize(dotpos);

    if (usedot)
    {
      s += ".0";
    }
  }
  else
  {
    // xxx.x(x)
    s.resize(dotpos + addtail + 1);
  }

  return s;
}