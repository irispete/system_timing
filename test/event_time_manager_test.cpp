
#include <iris_unit_test_framework/iris_unit_test_framework.h>
#define private public
#include <system_timing/event_time_publisher.h>
#include <system_timing/event_time_manager.h>
#include <iris_common/test/mock_logger.h>
#undef private

using namespace system_timing;
using ::testing::_;
using ::testing::Return;


class EventTimeManagerTests : public testing::Test
{
    public:
        MockLogger mock_logger_{};
        EventTimeManager manager_{mock_logger_};

        void SetUp()
        {
            manager_.setSaveFunction(std::bind(&EventTimeManagerTests::saveSequence, this, std::placeholders::_1));
//            manager_.setSaveFunction([this](std::list<iris_common::EventTime> &sequence) {
//                this->saveSequence(sequence);
//            });
        }

        void TearDown()
        {
        }

        void saveSequence(std::list<iris_common::EventTime> &sequence)
        {
            sequence_ = sequence;
        }

        iris_common::EventTime makeEvent(const std::string &node, const std::string &event, const std::string &category, int id, EventTimeType event_type, int time)
        {
            iris_common::EventTime event_time{};
            event_time.node.data = node;
            event_time.event.data = event;
            event_time.category.data = category;
            event_time.id = id;
            event_time.type = static_cast<int>(event_type);
            event_time.time = time;
            return std::move(event_time);
        }

        int call_count = 0;
        int event_count_ = 0;
        std::list<iris_common::EventTime> sequence_;

};

/**
 * @brief
 */
TEST_F(EventTimeManagerTests, AddOneEventTest)
{
    auto event = makeEvent("node", "first", "EO", 111, EventTimeType::start, 123);
    manager_.addEvent(event);
    manager_.flush();
    ASSERT_EQ(sequence_.size(), 1u);
    ASSERT_EQ(mock_logger_.error_count, 0);
}

TEST_F(EventTimeManagerTests, AddTwoEventTest)
{
    auto event = makeEvent("node", "first", "EO", 111, EventTimeType::start, 123);
    manager_.addEvent(event);
    manager_.flush();
    ASSERT_EQ(sequence_.size(), 1u);
    ASSERT_EQ(mock_logger_.error_count, 0);

    event = makeEvent("node", "first", "EO", 111, EventTimeType::start, 123);
    manager_.addEvent(event);
    manager_.flush();
    ASSERT_EQ(sequence_.size(), 1u);
    ASSERT_EQ(mock_logger_.error_count, 1);

    event = makeEvent("node", "add", "EO", 111, EventTimeType::add, 222);
    manager_.addEvent(event);
    manager_.flush();
    ASSERT_EQ(sequence_.size(), 2u);
    ASSERT_EQ(mock_logger_.error_count, 1);

    event = makeEvent("node", "add", "EO", 111, EventTimeType::add, 133);
    manager_.addEvent(event);
    manager_.flush();
    ASSERT_EQ(sequence_.size(), 3u);
    ASSERT_EQ(mock_logger_.error_count, 2);

}

/**
 * @brief
 */
TEST_F(EventTimeManagerTests, FindSequenceTest)
{
    auto event = makeEvent("node", "first", "EO", 111, EventTimeType::start, 123);
    auto sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence_.size(), 0u);
    manager_.sequences_[event.category.data][event.id].push_back(event);
    sequence = manager_.findSequence_(event);
    ASSERT_EQ(sequence.size(), 1u);
}

