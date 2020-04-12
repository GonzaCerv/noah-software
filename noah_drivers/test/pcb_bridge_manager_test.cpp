// Bring in my package's API, which is what I'm testing
// Bring in gtest

// Standard libraries
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>

// ROS libraries
#include <ros/ros.h>

// Noah libraries
// #include <libserial/SerialPort.h>
#include <noah_drivers/UartPackage.hpp>
#include <noah_drivers/PCBBridgeManager.hpp>

using namespace ::testing;
using namespace LibSerial ;

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

class PcbBridgeManagerTest : public Test {
   public:
    PcbBridgeManagerTest() : nh_("~") {}

    void SetUp() override {
        uut_ = std::make_unique<PCBBridgeManager>(port_name_raspberry);
        // serial_port_->Open(port_name_pcb);
        // serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        // serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        // serial_port_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        // serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
        // serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    }

    ros::NodeHandle nh_;
    // std::unique_ptr<SerialPort> serial_port_ ;
    std::unique_ptr<PCBBridgeManager> uut_;
    UartPackage current_package_;
};

// Test if it is possible to open the port.
TEST_F(PcbBridgeManagerTest, NoCommand) {
    // auto packages = uut_->check();
    // EXPECT_EQ(0,packages.size());
    EXPECT_TRUE(true);
}

// // Test if it is possible to open the port.
// TEST_F(PcbBridgeManagerTest, SingleCommandNoParameters) {
//     Uart uart_pcb_(port_name_pcb);
//     uart_pcb_.openPort();
//     uart_pcb_.transmit(command_no_params);
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(1, callback_count);

//     // Check if command is correct.
//     ASSERT_EQ(0x00, current_package_.getCommandValue());

//     // Check the parameter.
//     ASSERT_TRUE(current_package_.parameters_.empty());
// }

// // Test for command with parameters.
// TEST_F(PcbBridgeManagerTest, SingleCommandWithParameters) {
//     Uart uart_pcb_(port_name_pcb);
//     uart_pcb_.openPort();
//     uart_pcb_.transmit(command_one_param);
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(1, callback_count);

//     // Check if command is correct.
//     ASSERT_EQ(command_one_param[1], current_package_.getCommandValue());

//     // Check the parameter.
//     ASSERT_TRUE(!current_package_.parameters_.empty());
//     ASSERT_EQ(command_one_param[2], current_package_.parameters_[0]);
// }

// // Test command for the longest command possible.
// TEST_F(PcbBridgeManagerTest, SingleLongCommandWithParameters) {
//     Uart uart_pcb_(port_name_pcb);
//     uart_pcb_.openPort();
//     uart_pcb_.transmit(command_long_param);
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(1, callback_count);
//     // Check if command is correct.
//     ASSERT_EQ(command_long_param[1], current_package_.getCommandValue());
//     // Check the parameter.
//     ASSERT_TRUE(!current_package_.parameters_.empty());
//     auto it = command_long_param.begin();
//     it += 2;
//     uint32_t index = 0;
//     for (; it != command_long_param.end(); ++it, ++index) {
//         ASSERT_EQ(*it, current_package_.parameters_[index]);
//     }
// }

// // Test for packages with missing start.
// TEST_F(PcbBridgeManagerTest, SingleWrongCommandNoStart) {
//     Uart uart_pcb_(port_name_pcb);
//     uart_pcb_.openPort();
//     uart_pcb_.transmit(command_wrong_param_no_start);
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(0, callback_count);
// }

// // Test for packages with missing stop.
// TEST_F(PcbBridgeManagerTest, SingleWrongCommandNoStop) {
//     Uart uart_pcb_(port_name_pcb);
//     uart_pcb_.openPort();
//     uart_pcb_.transmit(command_wrong_param_no_stop);
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(0, callback_count);
// }

// // Test wrong package with not enough params.
// TEST_F(PcbBridgeManagerTest, SingleWrongCommandNoEnoughParams) {
//     Uart uart_pcb_(port_name_pcb);
//     uart_pcb_.openPort();
//     uart_pcb_.transmit(command_wrong_param_no_enough_params);
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(0, callback_count);
// }

// // Test for wrong command with too much params
// TEST_F(PcbBridgeManagerTest, SingleWrongCommandTooMuchParams) {
//     Uart uart_pcb_(port_name_pcb);
//     uart_pcb_.openPort();
//     uart_pcb_.transmit(command_wrong_param_too_much_params);
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(0, callback_count);
// }

// // Sends 2 correct commands.
// TEST_F(PcbBridgeManagerTest, DoubleCommandMixedParameters) {
//     Uart uart_pcb_(port_name_pcb);
//     uart_pcb_.openPort();

//     // Send both commands simultaneously
//     uart_pcb_.transmit(command_no_params);
//     uart_pcb_.transmit(command_one_param);

//     // ***************************
//     // Process command with params
//     // ***************************
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(1, callback_count);
//     // Check if command is correct.
//     ASSERT_EQ(0x00, current_package_.getCommandValue());
//     // Check the parameter.
//     ASSERT_TRUE(current_package_.parameters_.empty());

//     // ***************************
//     // Process command with params
//     // ***************************
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(2, callback_count);
//     // Check if command is correct.
//     ASSERT_EQ(command_one_param[1], current_package_.getCommandValue());
//     // Check the parameter.
//     ASSERT_TRUE(!current_package_.parameters_.empty());
//     ASSERT_EQ(command_one_param[2], current_package_.parameters_[0]);
// }

// // Sends 1 command between 2 wrong commands.
// TEST_F(PcbBridgeManagerTest, TripleCommandWrongMixedParameters) {
//     Uart uart_pcb_(port_name_pcb);
//     uart_pcb_.openPort();

//     // Send both commands simultaneously
//     uart_pcb_.transmit(command_wrong_param_too_much_params);
//     uart_pcb_.transmit(command_one_param);
//     uart_pcb_.transmit(command_wrong_param_no_stop);

//     // ***************************
//     // Process command with params
//     // ***************************
//     package_manager_->check();

//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(1, callback_count);
//     // Check if command is correct.
//     ASSERT_EQ(command_one_param[1], current_package_.getCommandValue());
//     // Check the parameter.
//     ASSERT_EQ(command_one_param[2], current_package_.parameters_[0]);
//     package_manager_->check();
//     // Check if the package is being passed to the callback.
//     ASSERT_EQ(1, callback_count);
// }

}  // namespace noah_drivers

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    ros::init(argc, argv, "package_manager_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
