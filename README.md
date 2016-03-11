# esp_mqtt_pms5003
This project is modified from https://github.com/tuanpmt/esp_mqtt
Made just few modification to fit LASS (test version) requirements.

The MQTT broker is user.jdaiot.com

Topic MUST be in this form
    /LASS/886/00/#
    
Global format
    /LASS/(country code)/(state code)/(city code)/(device id)/(application name)
    
For example,
    /LASS/886/00/06/g500001/PM2.5
