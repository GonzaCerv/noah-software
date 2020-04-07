// Bring in my package's API, which is what I'm testing
// Bring in gtest

// Standard libraries
#include <gtest/gtest.h>

// Noah libraries
#include <noah_drivers/Uart.hpp>

// ROS libraries
#include <ros/ros.h>

using namespace ::testing;

using noah_drivers::Uart;

class UartManagerTests : public ::testing::Test
{
};

// Test if it is possible to open the port.
TEST(UartManagerTests, openPort)
{
  Uart test_uart("/dev/pts/2");
  EXPECT_TRUE(test_uart.openPort());
}

// Test if it is possible to close the port.
TEST(UartManagerTests, closePort)
{
  Uart test_uart("/dev/pts/2");
  test_uart.openPort();
  EXPECT_TRUE(test_uart.closePort());
}

// Try to read when there is no data sent.
TEST(UartManagerTests, readNoData)
{
  Uart test_uart_2("/dev/pts/2");

  // Open ports
  test_uart_2.openPort();

  auto result = test_uart_2.receive();

  //Check if there is data.
  ASSERT_TRUE(result.empty());
}

// Try to send and receive a package. In this case,
// it reads the exact same amount of characters sent.
TEST(UartManagerTests, sendReceiveCorrectAmount)
{
  Uart test_uart_2("/dev/pts/2");
  Uart test_uart_3("/dev/pts/3");
  // Open ports
  test_uart_2.openPort();
  test_uart_3.openPort();

  // Test if possible to send data and get the response.
  std::vector<uint8_t> data;
  data.push_back('a');
  data.push_back('b');
  data.push_back('c');

  test_uart_2.transmit(data);
  auto result = test_uart_3.receive(3);

  //Check if there is data.
  ASSERT_TRUE(!result.empty());

  //Check if the values matches.
  ASSERT_EQ(data,result);

  ASSERT_TRUE(true);
}

// Try to send and receive a package. In this case,
// it reads more characters that the one sent.
TEST(UartManagerTests, sendReceiveMoreReadThanSent)
{
  Uart test_uart_2("/dev/pts/2");
  Uart test_uart_3("/dev/pts/3");
  // Open ports
  test_uart_2.openPort();
  test_uart_3.openPort();

  // Test if possible to send data and get the response.
  std::vector<uint8_t> data;
  data.push_back('a');
  data.push_back('b');
  data.push_back('c');

  test_uart_2.transmit(data);
  auto result = test_uart_3.receive(10);

  //Check if there is data.
  ASSERT_TRUE(!result.empty());

  //Check if the values matches.
  ASSERT_EQ(data,result);
}

// Try to send and receive a package. In this case,
// it reads the max amount of characters.
TEST(UartManagerTests, sendReceiveAll)
{
  Uart test_uart_2("/dev/pts/2");
  Uart test_uart_3("/dev/pts/3");
  // Open ports
  test_uart_2.openPort();
  test_uart_3.openPort();

  // Test if possible to send data and get the response.
  std::vector<uint8_t> data;
  data.push_back('a');
  data.push_back('b');
  data.push_back('c');

  test_uart_2.transmit(data);
  auto result = test_uart_3.receive();

  //Check if there is data.
  ASSERT_TRUE(!result.empty());

  //Check if the values matches.
  ASSERT_EQ(data,result);
}

// Try to send and receive a package. In this case,
// it reads one character at a time.
TEST(UartManagerTests, sendReceiveJustOne)
{
  Uart test_uart_2("/dev/pts/2");
  Uart test_uart_3("/dev/pts/3");
  // Open ports
  test_uart_2.openPort();
  test_uart_3.openPort();

  // Test if possible to send data and get the response.
  std::vector<uint8_t> data;
  data.push_back('a');
  data.push_back('b');

  test_uart_2.transmit(data);

  // Reads just 1 character.
  auto result = test_uart_3.receive(1);
  //Check if there is data.
  ASSERT_TRUE(!result.empty());
  //Check if the values matches.
  ASSERT_EQ(data[0],result[0]);

  // Reads the next character.
  result = test_uart_3.receive(1);
  //Check if there is data.
  ASSERT_TRUE(!result.empty());
  //Check if the values matches.
  ASSERT_EQ(data[1],result[0]);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "uart_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
