# esp_mqtt_pms5003
This project is modified from https://github.com/tuanpmt/esp_mqtt
Made just few modification to fit DESN (test version) requirements.

The MQTT broker is user.jdaiot.com

Topic MUST be in this form
    /DESN/886/00/#
    
Global format
    /DESN/(country code)/(state code)/(city code)/(device id)/(application name)
    
For example,
    /DESN/886/00/06/g500001/pm2.5
    /DESN/886/00/p000001/pm2.5
