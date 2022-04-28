#ifndef CYBERDOG_OTA_OTA_OPTIONS_HPP_
#define CYBERDOG_OTA_OTA_OPTIONS_HPP_


#include <memory>
#include <string>
#include <vector>

namespace cyberdog {

const std::string kOTAGrpcServerName = "ota_grpc";
const std::string kOTAReadyRequestTopic = "ota_ready_request";
const std::string kOTAReadyResponseTopic = "ota_ready_response";
const std::string kOTABmsStatusTopic = "bms_status";

const std::string kOTACommandStatusQuery = "ota_command_status_query";
const std::string kOTACommandVersionQuery =  "ota_command_version_query";
const std::string kOTACommandProcessQuery =  "ota_command_process_query";
const std::string kOTACommandStartUpgrade = "ota_command_start_upgrade";
const std::string kOTACommandStartDownload = "ota_command_start_download";
const std::string kOTACommandEstimateUpgradeTimeQuery = "ota_command_estimate_upgrade_time_query";


void CreateOTAOptions();

}  // cyberdog


#endif  // CYBERDOG_OTA_OTA_OPTIONS_HPP_
