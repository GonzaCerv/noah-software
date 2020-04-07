// Bring in my package's API, which is what I'm testing
// Bring in gtest

// Standard libraries
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>

// Noah libraries
#include <noah_drivers/PackageManager.hpp>
#include <noah_drivers/Uart.hpp>
#include <noah_drivers/UartPackage.hpp>

// ROS libraries
#include <ros/ros.h>

using namespace ::testing;

namespace noah_drivers {

const std::string port_name_raspberry = "/dev/pts/2";
const std::string port_name_pcb = "/dev/pts/3";
const std::vector<uint8_t> command_no_params = {0xA0, 0x00, 0xB0};
const std::vector<uint8_t> command_one_param = {0xA0, 0x10, 90, 0xB0};
const std::vector<uint8_t> command_wrong_param_no_start = {0x00, 0x10, 90, 0xB0};
const std::vector<uint8_t> command_wrong_param_no_stop = {0xA0, 0x10, 90, 0xF0};
const std::vector<uint8_t> command_wrong_param_no_enough_params = {0xA0, 0x10, 0xF0};
const std::vector<uint8_t> command_wrong_param_too_much_params = {0xA0, 0x10, 0xA0, 0xA0, 0xB0};
const std::vector<uint8_t> command_long_param = {0xA0, 0x81, 0x10, 0x20, 0x40, 0x80, 0x10, 0x20, 0x10, 0x20, 0xB0};

class PackageManagerTests : public Test {
   public:
    PackageManagerTests() : nh_("~"), uart_(port_name_raspberry) {}

    void SetUp() override {
        callback_count = 0;
        auto callback = std::bind(&PackageManagerTests::execute_callback, this, std::placeholders::_1);
        package_manager_ = std::make_shared<PackageManager>(uart_, callback);
        package_manager_->start();
    }

    void execute_callback(const UartPackage& package) {
        ++callback_count;
        current_package_ = package;
    }

    ros::NodeHandle nh_;
    Uart uart_;
    std::shared_ptr<PackageManager> package_manager_;
    UartPackage current_package_;
    int callback_count;
};

// Test if it is possible to open the port.
TEST_F(PackageManagerTests, NoCommand) {
    package_manager_->check();
    EXPECT_EQ(0, callback_count);
}

// Test if it is possible to open the port.
TEST_F(PackageManagerTests, SingleCommandNoParameters) {
    Uart uart_pcb_(port_name_pcb);
    uart_pcb_.openPort();
    uart_pcb_.transmit(command_no_params);
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(1, callback_count);

    // Check if command is correct.
    ASSERT_EQ(0x00, current_package_.getCommandValue());

    // Check the parameter.
    ASSERT_TRUE(current_package_.parameters_.empty());
}

// Test for command with parameters.
TEST_F(PackageManagerTests, SingleCommandWithParameters) {
    Uart uart_pcb_(port_name_pcb);
    uart_pcb_.openPort();
    uart_pcb_.transmit(command_one_param);
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(1, callback_count);

    // Check if command is correct.
    ASSERT_EQ(command_one_param[1], current_package_.getCommandValue());

    // Check the parameter.
    ASSERT_TRUE(!current_package_.parameters_.empty());
    ASSERT_EQ(command_one_param[2], current_package_.parameters_[0]);
}

// Test command for the longest command possible.
TEST_F(PackageManagerTests, SingleLongCommandWithParameters) {
    Uart uart_pcb_(port_name_pcb);
    uart_pcb_.openPort();
    uart_pcb_.transmit(command_long_param);
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(1, callback_count);
    // Check if command is correct.
    ASSERT_EQ(command_long_param[1], current_package_.getCommandValue());
    // Check the parameter.
    ASSERT_TRUE(!current_package_.parameters_.empty());
    auto it = command_long_param.begin();
    it += 2;
    uint32_t index = 0;
    for (; it != command_long_param.end(); ++it, ++index) {
        ASSERT_EQ(*it, current_package_.parameters_[index]);
    }
}

// Test for packages with missing start.
TEST_F(PackageManagerTests, SingleWrongCommandNoStart) {
    Uart uart_pcb_(port_name_pcb);
    uart_pcb_.openPort();
    uart_pcb_.transmit(command_wrong_param_no_start);
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(0, callback_count);
}

// Test for packages with missing stop.
TEST_F(PackageManagerTests, SingleWrongCommandNoStop) {
    Uart uart_pcb_(port_name_pcb);
    uart_pcb_.openPort();
    uart_pcb_.transmit(command_wrong_param_no_stop);
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(0, callback_count);
}

// Test wrong package with not enough params.
TEST_F(PackageManagerTests, SingleWrongCommandNoEnoughParams) {
    Uart uart_pcb_(port_name_pcb);
    uart_pcb_.openPort();
    uart_pcb_.transmit(command_wrong_param_no_enough_params);
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(0, callback_count);
}

// Test for wrong command with too much params
TEST_F(PackageManagerTests, SingleWrongCommandTooMuchParams) {
    Uart uart_pcb_(port_name_pcb);
    uart_pcb_.openPort();
    uart_pcb_.transmit(command_wrong_param_too_much_params);
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(0, callback_count);
}

// Sends 2 correct commands.
TEST_F(PackageManagerTests, DoubleCommandMixedParameters) {
    Uart uart_pcb_(port_name_pcb);
    uart_pcb_.openPort();

    // Send both commands simultaneously
    uart_pcb_.transmit(command_no_params);
    uart_pcb_.transmit(command_one_param);

    // ***************************
    // Process command with params
    // ***************************
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(1, callback_count);
    // Check if command is correct.
    ASSERT_EQ(0x00, current_package_.getCommandValue());
    // Check the parameter.
    ASSERT_TRUE(current_package_.parameters_.empty());

    // ***************************
    // Process command with params
    // ***************************
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(2, callback_count);
    // Check if command is correct.
    ASSERT_EQ(command_one_param[1], current_package_.getCommandValue());
    // Check the parameter.
    ASSERT_TRUE(!current_package_.parameters_.empty());
    ASSERT_EQ(command_one_param[2], current_package_.parameters_[0]);
}

// Sends 1 command between 2 wrong commands.
TEST_F(PackageManagerTests, TripleCommandWrongMixedParameters) {
    Uart uart_pcb_(port_name_pcb);
    uart_pcb_.openPort();

    // Send both commands simultaneously
    uart_pcb_.transmit(command_wrong_param_too_much_params);
    uart_pcb_.transmit(command_one_param);
    uart_pcb_.transmit(command_wrong_param_no_stop);

    // ***************************
    // Process command with params
    // ***************************
    package_manager_->check();

    // Check if the package is being passed to the callback.
    ASSERT_EQ(1, callback_count);
    // Check if command is correct.
    ASSERT_EQ(command_one_param[1], current_package_.getCommandValue());
    // Check the parameter.
    ASSERT_EQ(command_one_param[2], current_package_.parameters_[0]);
    package_manager_->check();
    // Check if the package is being passed to the callback.
    ASSERT_EQ(1, callback_count);
}

}  // namespace noah_drivers

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    ros::init(argc, argv, "package_manager_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
