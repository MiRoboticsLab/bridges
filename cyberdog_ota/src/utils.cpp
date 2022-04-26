#include "cyberdog_ota/utils.hpp"

namespace cyberdog
{

void Strrevn(char* s, const int len)
{
  char* h = s;
  char* t = s + len - 1;
  char ch;

  while (h < t) {
      ch = *h;
      *h++ = *t;    
      *t-- = ch;   
  }
}


long long FileSize(const char* path)
{
    FILE* fp = fopen(path, "rb");

    if (NULL == fp) {
        return -1;
    }


    if (fseek(fp, 0, SEEK_END)) {
        fclose(fp);
        return -2;
    }

    long long file_size = ftell(fp);
    fclose(fp);
    return file_size;
}

}