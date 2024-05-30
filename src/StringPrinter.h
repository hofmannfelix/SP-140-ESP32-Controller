#ifndef STRINGPRINTER_H
#define STRINGPRINTER_H

#include <Arduino.h>

class StringPrinter {
  private:
    static const size_t MAX_BUFFER_SIZE = 1024;
    static const size_t CHUNK_SIZE = 20;
    String buffer;

  public:
    StringPrinter();

    void print(const String &s);
    void print(const char* s);
    void print(char c);
    void print(int n);
    void print(unsigned int n);
    void print(long n);
    void print(unsigned long n);
    void print(float n);
    void print(double n);

    void println();
    void println(const String &s);
    void println(const char* s);
    void println(char c);
    void println(int n);
    void println(unsigned int n);
    void println(long n);
    void println(unsigned long n);
    void println(float n);
    void println(double n);

    String getBuffer();
    void clear();
    String readChunk();
    void deleteChunk();
};

#endif // STRINGPRINTER_H
