#include "Logger.h"

Logger::Logger(){

}

void Logger::setup(){
    SD.begin(BUILTIN_SDCARD);

    // get previously used run number, use incremented and then save it back into EEPROM
    byte runNo = EEPROM.read(RUN_NO_ADDR);
    // if runNo == 255, then we need to reset it since byte's max size is 255
    if(runNo == 255){
        runNo = 0;
        path = "run0/";
    }
    else{
        path = "run" + String(++runNo) + "/";
    }
    SD.mkdir(path.c_str());
    EEPROM.write(RUN_NO_ADDR, runNo);
}

void Logger::initializeFile(String filename, std::vector<String> columns){
    
    // create SDMFile object
    SDMFile sdmFile;
    sdmFile.name = path + filename + ".csv";
    sdmFile.currentRow.resize(columns.size() + 1); // add an extra column for timestamp
    sdmFile.previousRow.resize(columns.size() + 1);

    // initialize
    for(size_t i = 0; i < sdmFile.currentRow.size(); i++){
        sdmFile.currentRow[i] = SENTINEL_VAL;
        sdmFile.previousRow[i] = SENTINEL_VAL;
    }

    // setup columns
    sdmFile.columns.insert({"timestamp (s)", 0});
    for(size_t i = 0; i < columns.size(); i++){
        sdmFile.columns.insert({columns[i], i+1});
    }
    
    // create csv header
    File file = SD.open(sdmFile.name.c_str(), FILE_WRITE);
    if(file){
        file.print("timestamp (s)");
        for(auto& c : columns){
            file.print(", " + c);
        }
        file.println();
        file.close();
    }

    // save it to our hashmap
    files.insert({sdmFile.name, sdmFile});
}

void Logger::initializeLogFile(String filename){
    String fullpath = path + filename + ".log";
    File file = SD.open(fullpath.c_str(), FILE_WRITE);
    if(file){
        String line = String(millis() / 1000.0, 2) + String(": [STA] Initializing log file");
        file.println(line);
    }
    file.close();
}

bool Logger::log(String filename, LogLevel level, String topic, String message){
    String fullpath = path + filename + ".log";
    File file = SD.open(fullpath.c_str(), FILE_WRITE);
    if(file){
        float timestamp = millis() / 1000.0;
        String type = "";
        switch(level){
            case LogLevel::Status:
            type = "[STA]";
            break;
            case LogLevel::Warning:
            type = "[WRN]";
            break;
            case LogLevel::Error:
            type = "[ERR]";
            break;
            default:
            type = "[NTP]";
            break;
        }
        String line = String(timestamp, 2) + " " + type + topic + ": " + message;
        file.println(line);
        file.close();

        return true;
    }
    file.close();

    return false;
}

bool Logger::addData(String filename, String column, float data){
    String key = path + filename + ".csv";
    auto pair = files[key].columns.find(column);
    if(pair != files[key].columns.end()){
        files[key].currentRow[pair->second] = data;
        return true;
    }
    else return false;
    
}

bool Logger::writeRow(String filename){
    String key = path + filename + ".csv";
    File file = SD.open(key.c_str(), FILE_WRITE);
    if(file){
        // first, make timestamp
        files[key].currentRow[0] = millis() / 1000.0;

        // if a data thing is missing, use previousRow data for it
        for(size_t i = 0; i < files[key].currentRow.size(); i++){
            if(files[key].currentRow[i] == SENTINEL_VAL){
                files[key].currentRow[i] = files[key].previousRow[i];
            }
        }

        // print data
        for(size_t i = 0; i < files[key].currentRow.size(); i++){
            // if latitude or longitude, print with 4 decimal places
            // both will be the last two in the list
            if (files[key].currentRow.size() - i <= 2) {
                file.print(files[key].currentRow[i], 4);
            }
            else {
                file.print(files[key].currentRow[i]);
            }
            
            // if we arent at the last column, print a comma
            if(i != files[key].currentRow.size() - 1){
                file.print(",");
            }
        }
        file.println();
        file.close();

        // copy currentRow to previousRow
        for(size_t i = 0; i < files[key].previousRow.size(); i++){
            files[key].previousRow[i] = files[key].currentRow[i];
        }
        // reset currentRow
        for(size_t i = 0; i < files[key].currentRow.size(); i++)
            files[key].currentRow[i] = SENTINEL_VAL;
        
        return true;
    }
    else return false;
}
