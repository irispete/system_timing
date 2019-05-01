#include <vector>
#include <boost/filesystem.hpp>
#include <chrono>
#include <ctime>
#include <iris_unit_test_framework/iris_unit_test_framework.h>
#include <iris_common/log/csv_logger.h>
#include <system_timing/event_time_csv_logger.h>
#include <iris_common/log/csv_logger_inc.h>
#include <iris_common/EventTime.h>
#include <ros/ros.h>

using namespace system_timing;


class EventTImerCsvLoggerTest : public testing::Test
{
    protected:
        std::string path_ = "events.csv";
        EventTimeCsvLogger logger_;
        const unsigned int header_size = 50;

    virtual void SetUp()
    {
        boost::filesystem::remove(path_);
        logger_.initialize(path_);
    }

    virtual void TearDown()
    {
        //boost::filesystem::remove(path_);
    }

    iris_common::EventTime makeEvent()
    {
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
        iris_common::EventTime event_time{};
        event_time.node.data = "test-node";
        event_time.event.data = "frame-write";
        event_time.category.data = "EO";
        event_time.id = 111;
        event_time.type = 2;
        event_time.time = now_ms;
        return std::move(event_time);
    }

};

/**
 * @brief Verifies the file at path_ of the test class exists
 */
TEST_F(EventTImerCsvLoggerTest, FileCreationTest)
{
    ASSERT_TRUE(boost::filesystem::exists(path_));
}

/**
 * @brief Verifies the number of items read in a single line equals the expected number of items
 */
TEST_F(EventTImerCsvLoggerTest, HeaderLengthTest)
{
    // Read a single line from the test csv file
    std::ifstream input(path_);
    std::string line;
    std::getline(input, line);

    // Split string by the comma, inserting each item into a vector
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> tokens;
    while (std::getline(ss, item, ',')) {
        tokens.push_back(item);
    }
    input.close();

    // Verify number of items read is equal to number expected
    ASSERT_EQ(tokens.size(), header_size);
}

/**
 * @brief Verifies the number of items wrote in a single line equals the expected number of items
 */
TEST_F(EventTImerCsvLoggerTest, LengthWrittenLineTest)
{
    auto event = makeEvent();
    std::list<iris_common::EventTime> sequence{event};
    auto event2 = makeEvent();
    sequence.push_back(event2);

    // Write to the file
    EXPECT_TRUE(logger_.write(sequence));

    std::ifstream input(path_);
    std::string line;

    // Get header line first
    std::getline(input, line);

    // Get next line
    std::getline(input, line);

    // Split string by the comma, inserting each item into a vector
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> tokens;
    while (std::getline(ss, item, ',')) {
        tokens.push_back(item);
    }
    input.close();

    // Verify number of items on that line is equal to number expected
    ASSERT_EQ(tokens.size(), 10u);
}

/**
 * @brief Verifies writing one line writes only one line (small number of lines)
 */
TEST_F(EventTImerCsvLoggerTest, WriteLineToFileSmallTest)
{
    auto event = makeEvent();
    std::list<iris_common::EventTime> sequence{event};

    // Write to the file 5 rimes
    for(int i = 0; i < 5; i++)
    {
        EXPECT_TRUE(logger_.write(sequence));
    }

    int counter = 0;
    std::ifstream input(path_);
    std::string line;

    // Get next line
    std::getline(input, line);

    // Loop until EOF, counting each line
    while(std::getline(input, line))
    {
        ++counter;
    }

    // Verify the correct number of lines was written
    ASSERT_EQ(5, counter);
}

/**
 * @brief Verifies writing one line writes only one line (large number of lines)
 */
TEST_F(EventTImerCsvLoggerTest, WriteLineToFileLargeTest)
{
    auto event = makeEvent();
    std::list<iris_common::EventTime> sequence{event};

    // Write to the file 1000 rimes
    for(int i = 0; i < 1000; i++)
    {
        EXPECT_TRUE(logger_.write(sequence));
    }

    int counter = 0;
    std::ifstream input(path_);
    std::string line;

    // Get header line first
    std::getline(input, line);

    // Loop until EOF, counting each line
    while(std::getline(input, line))
    {
        ++counter;
    }

    // Verify the correct number of lines was written
    ASSERT_EQ(1000, counter);
}

