#ifndef MCU_FILE_H
#define MCU_FILE_H

#include "limesuiteng/config.h"

#include <stdio.h>
#include <cstring>
#include <vector>
#include <string>

/** @brief Structure describing a memory block. */
struct MemBlock {
    size_t m_startAddress; ///< The start address of this memory block.
    std::vector<unsigned char> m_bytes; ///< The bytes contained in this block of memory.

    /// @brief Returns the end address of this memory block.
    /// @return The end address of the memory block.
    size_t GetEndAddress() const { return m_startAddress + m_bytes.size() - 1; }
};

/// @brief Structure for reading an MCU file.
class LIME_API MCU_File
{
  public:
    /// @brief Opens a given file name for reading in the given mode.
    /// @param fileName The name of the file to read the memory from.
    /// @param mode The mode with which to open the file (as used by `fopen`)
    explicit MCU_File(const char* fileName, const char* mode);
    ~MCU_File();

    /// @brief Checks if a file is opened.
    /// @return True if a file is opened.
    bool FileOpened();

    /// @brief Reads the file as a raw memory file.
    /// @param limit The maximum amount of bytes to read.
    void ReadBin(unsigned long limit);

    /// @brief Reads the file as a hexadecimal values file.
    /// @param limit The maximum amount of bytes to read.
    void ReadHex(unsigned long limit);

    /// @brief Gets the byte of the specified memory address.
    /// @param address The address of the byte to get.
    /// @param chr The parameter to where the byte will be read to.
    /// @return True if a value was stored in `chr`.
    bool GetByte(const unsigned long address, unsigned char& chr);

    /// @brief Gets the sequence of bits as a string of '0's and '1's
    /// @param address The address to start reading from.
    /// @param bits The length of the bit string (will break if exceeding `unsigned long` limits)
    /// @param lEndian Whether the data is in Little-Endian or not (true - LE, false - BE)
    /// @param str The output string.
    /// @return True if a value was stored in `str`.
    bool BitString(const unsigned long address, const unsigned char bits, const bool lEndian, std::string& str);

    /// @brief Returns a handle to the file
    /// @return The handle to the file.
    FILE* Handle() const { return m_file; };

    /// @brief The read chunks of memory.
    std::vector<MemBlock> m_chunks;

  private:
    FILE* m_file;
};

#endif // MCU_FILE_H