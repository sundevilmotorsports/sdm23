/**
 * @file Logger.h
 * @author Joshua Tenorio
 * @brief Contains prototypes for Logger class
 */
#pragma once

#include <Arduino.h>
#include <SD.h>
#include <EEPROM.h>
#include <vector>
#include <map>

#define RUN_NO_ADDR     0
#define SENTINEL_VAL    0.0

// data structure used to represent a csv file
// name: complete filename w/ path
// columns: what the columns are named. includes timestamp by default
// currentRow: the current row we're making
typedef struct SDMFile_s_t {
    String name;
    std::map<String, size_t> columns;
    std::vector<float> currentRow;
    std::vector<float> previousRow;
} SDMFile;

enum class LogLevel {
    Status,
    Warning,
    Error
};

class Logger {
    public:
    Logger();

    // TODO: move functions in Logger to here
    void setup();

    // creates a .csv file and SDMFile object with desired columns
    // TODO: change this to initializePlotFile
    void initializeFile(String filename, std::vector<String> columns);

    // creates a .log file where we can log messages
    void initializeLogFile(String filename);

    // add a log message
    bool log(String filename, LogLevel level, String topic, String message);

    // adds a data entry to filename's column
    // filename: name of the file, doesnt need to be the complete path
    bool addData(String filename, String column, float data);

    // writes currentRow into the actual csv file
    bool writeRow(String filename);

    protected:
    // folder to write into
    String path;

    // hashmap for SDMFile objects
    std::map<String, SDMFile> files;
};