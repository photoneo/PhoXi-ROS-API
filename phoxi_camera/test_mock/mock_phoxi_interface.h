#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "phoxi_camera/PhoXiInterface.h"

using ComponentList = std::vector<std::pair<std::string, bool>>;
using SettingValueMap = std::map<std::string, phoxi_camera::SettingValue>;
using SettingKeyValueList = std::vector<std::pair<std::string, phoxi_camera::SettingValue>>;

class MockPhoXiInterface : public phoxi_camera::PhoXiInterface {
public:
    MOCK_METHOD(void, connectCamera, (const std::string&, GetFrameCallback&&), (override));
    MOCK_METHOD(void, disconnectCamera, (), (override));
    MOCK_METHOD(void, triggerFrame, (bool), (override));
    MOCK_METHOD(void, startAcquisition, (), (override));
    MOCK_METHOD(void, stopAcquisition, (), (override));
    MOCK_METHOD(bool, isConnected, (), (override));
    MOCK_METHOD(bool, isAcquiring, (), (override));
    MOCK_METHOD(void, setFrameOutputSettings, (const ComponentList&), (override));
    MOCK_METHOD(std::vector<pho::api::PhoXiProfileDescriptor>, getProfileList, (), (override));
    MOCK_METHOD(std::string, getActiveProfile, (), (override));
    MOCK_METHOD(void, setActiveProfile, (const std::string&), (override));
    MOCK_METHOD(std::string, getStartupProfile, (), (override));
    MOCK_METHOD(void, setStartupProfile, (const std::string&), (override));
    MOCK_METHOD(void, createProfile, (const std::string&), (override));
    MOCK_METHOD(void, deleteProfile, (const std::string&), (override));
    MOCK_METHOD(void, updateProfile, (const std::string&), (override));
    MOCK_METHOD(pho::api::PhoXiProfileContent, exportProfile, (), (override));
    MOCK_METHOD(void, importProfile, (const pho::api::PhoXiProfileContent&), (override));
    MOCK_METHOD(void, resetActiveProfile, (), (override));
    MOCK_METHOD(phoxi_camera::PhoXiDeviceInformation, getDeviceInfo, (), (override));
    MOCK_METHOD(std::vector<phoxi_camera::SettingInfo>, getSettingInfos, (), (const, override));
    MOCK_METHOD(std::vector<phoxi_camera::FrameComponentInfo>, getFrameComponentInfos, (), (const, override));
    MOCK_METHOD((std::map<std::string, bool>), getFrameOutputSettings, (const std::vector<std::string>&), (override));
    MOCK_METHOD(phoxi_camera::SettingValue, getSetting, (const std::string&), (override));
    MOCK_METHOD(SettingValueMap, getSettings, (const std::vector<std::string>&), (override));
    MOCK_METHOD(void, setSetting, (const std::string&, const phoxi_camera::SettingValue&), (override));
    MOCK_METHOD(void, setSettings, (const SettingKeyValueList&), (override));
    MOCK_METHOD(void, rebootDevice, (const std::string&), (override));
    MOCK_METHOD(void, shutdownDevice, (const std::string&), (override));
    MOCK_METHOD(void, factoryResetDevice, (const std::string&), (override));
    MOCK_METHOD(void, downloadDeviceLog, (const std::string&, const std::string&, bool), (override));
};
