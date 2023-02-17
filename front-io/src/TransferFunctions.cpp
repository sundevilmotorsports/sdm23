
/*
    Converts the output to pressure. 
    @returns brakePressure
 */
float transferBrakePressure(float voltage){
    float supplyVoltage = 3.3;
    float minPressure = 10;
    float maxPressure = 50;
    return (voltage - 0.10*supplyVoltage)*(maxPressure - minPressure)/(0.8 * supplyVoltage) + minPressure;
}