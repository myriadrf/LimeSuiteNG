#ifndef LIME_LA9310_H
#define LIME_LA9310_H

#include <string>

namespace lime {

class LA9310
{
  public:
    LA9310(const std::string& device_path);
    ~LA9310();

  private:
    const std::string mDevice_path;
    int mFileDescriptor;
};

} // namespace lime

#endif // LIME_LA9310_H
