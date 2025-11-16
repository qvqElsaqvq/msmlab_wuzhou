//
// Created by msmlab on 2025/11/14.
//

#ifndef FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H
#define FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H

#include <iostream>
#include <fstream>
#include <mqtt/async_client.h>

#include "message.h"
#include "cpp/INIReader.h"
#include "serial.h"

class MQTTServer : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener {
    void on_failure(const mqtt::token &tok) override {
        std::cout << "Connection failed!" << std::endl;
    }

    void on_success(const mqtt::token &tok) override {
        std::cout << "Connection success!" << std::endl;
    }

    void connection_lost(const std::string &cause) override {
        std::cout << "Connection lost: " << cause << std::endl;
    }

    void message_arrived(mqtt::const_message_ptr msg) override {
        std::cout << "On topic: " << msg->get_topic() <<" received message: " << msg->to_string() << std::endl;
    }

    void delivery_complete(mqtt::delivery_token_ptr tok) override {
        std::cout << "Delivery complete!" << std::endl;
    }
};

#endif //FAN_CONTROL_CODE_ADAPT_C_MQTT_SERVER_H