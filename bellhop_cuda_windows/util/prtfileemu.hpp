#pragma once

#ifndef _BHC_INCLUDING_COMPONENTS_
#error "Must be included from common.hpp!"
#endif

namespace bhc {

struct bhcInternal;

class PrintFileEmu {
public:
    PrintFileEmu(
        bhcInternal *internal, const std::string &FileRoot,
        void (*prtCallback)(const char *message))
        : callback(nullptr)
    {
        if(prtCallback == nullptr) {
            ofs.open(FileRoot + ".prt");
            if(!ofs.good()) {
                ExternalError(
                    internal, "Could not open print file: %s.prt", FileRoot.c_str());
            }
            ofs << std::unitbuf;
        } else {
            callback = prtCallback;
        }
    }
    ~PrintFileEmu()
    {
        if(ofs.is_open()) ofs.close();
    }

    template<typename T> PrintFileEmu &operator<<(const T &x)
    {
        if(callback != nullptr) {
            linebuf << x;
            if(linebuf.str().length() > 0) {
                callback(linebuf.str().c_str());
                linebuf.str("");
            }
        } else if(ofs.good()) {
            ofs << x;
        }
        return *this;
    }

private:
    std::ofstream ofs;
    std::stringstream linebuf;
    void (*callback)(const char *message);
};

} // namespace bhc
