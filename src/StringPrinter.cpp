#include "StringPrinter.h"

StringPrinter::StringPrinter() {
  buffer = "";
}

// Read up to CHUNK_SIZE bytes from the buffer
String StringPrinter::readChunk() {
  if (buffer.length() <= CHUNK_SIZE) {
    String chunk = buffer;
    clear(); // Clear the buffer since we've read everything
    return chunk;
  } else {
    String chunk = buffer.substring(0, CHUNK_SIZE);
    buffer = buffer.substring(CHUNK_SIZE);
    Serial.print("Log Buffer size is ");
    Serial.println(buffer.length());
    return chunk;
  }
}

void StringPrinter::print(const String &s) {
  if (buffer.length() + s.length() <= MAX_BUFFER_SIZE) {
    buffer += s;
  } else {
    buffer = "..." + buffer.substring(min(buffer.length() - 1, s.length() + 3)) + s;
  }
  Serial.print(s);
}

void StringPrinter::print(const char* s) {
  print(String(s));
}

void StringPrinter::print(char c) {
  print(String(c));
}

void StringPrinter::print(int n) {
  print(String(n));
}

void StringPrinter::print(unsigned int n) {
  print(String(n));
}

void StringPrinter::print(long n) {
  print(String(n));
}

void StringPrinter::print(unsigned long n) {
  print(String(n));
}

void StringPrinter::print(float n) {
  print(String(n));
}

void StringPrinter::print(double n) {
  print(String(n));
}

void StringPrinter::println() {
  print("\n");
}

void StringPrinter::println(const String &s) {
  print(s + "\n");
}

void StringPrinter::println(const char* s) {
  print(String(s) + "\n");
}

void StringPrinter::println(char c) {
  print(String(c) + "\n");
}

void StringPrinter::println(int n) {
  print(String(n) + "\n");
}

void StringPrinter::println(unsigned int n) {
  print(String(n) + "\n");
}

void StringPrinter::println(long n) {
  print(String(n) + "\n");
}

void StringPrinter::println(unsigned long n) {
  print(String(n) + "\n");
}

void StringPrinter::println(float n) {
  print(String(n) + "\n");
}

void StringPrinter::println(double n) {
  print(String(n) + "\n");
}

String StringPrinter::getBuffer() {
  return buffer;
}

void StringPrinter::clear() {
  buffer = "";
}
